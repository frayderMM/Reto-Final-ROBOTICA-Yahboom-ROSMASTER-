"""Logica pura (sin ROS2) de la maquina de estados ``unique_line``:
seguimiento de UNA sola pared lateral (derecha o izquierda) con LiDAR,
para chasis tipo Ackermann (no rota sobre su propio eje).

Funciones y clases sin estado de nodo, pensadas para validarse en
``unique_line_simulator.py`` y despues portarse tal cual (mismo
nombre de parametros, misma logica) a
``capytown_granprix/capytown_granprix/unique_line_node.py`` -- mismo
patron que ``wall_follow_control.py`` / ``turn_control.py`` en este
mismo directorio.

Idea central de la FSM (evita "girar hasta que el frente este libre",
que corta las esquinas en diagonal):

- FOLLOW_WALL: avanza recto corrigiendo con Kp hacia ``target_wall_dist``
  usando la pared lateral. Los giros de pared "hacia afuera" (la pared
  se curva alejandose del robot, ej. un tramo que gira hacia el lado
  contrario) NO necesitan un estado aparte: el control continuo de
  angulo+distancia sigue la curva solo. Unicamente los giros "hacia
  adentro" (pared que corta el paso al frente) necesitan un giro
  discreto.
- INTERIOR_TURN_90: gira ALEJANDOSE de la pared seguida (frente
  bloqueado -- esquina interior, pared saliente o fondo de corredor).
- EXTERIOR_CLEAR: cuando la pared lateral desaparece y el frente sigue
  libre (esquina exterior), avanza recto un tramo corto ANTES de girar
  -- si se gira de inmediato se corta la esquina en diagonal.
- EXTERIOR_TURN_90: gira HACIA la pared seguida, para recuperarla del
  otro lado de la esquina exterior.
- CORNER_ALIGN: tras cualquier giro, estabiliza contra la pared real
  (o mantiene rumbo si la pared todavia no aparece) antes de volver a
  FOLLOW_WALL -- evita reanudar el seguimiento con un angulo torcido.
- EMERGENCY_STOP: prioridad maxima en cualquier estado, frente
  demasiado cerca -> parar y esperar varias lecturas seguras antes de
  reanudar el estado en el que estaba.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

HALF_PI = math.pi / 2.0


# ----------------------------------------------------------------------
# Utilidades de angulo (duplicadas a proposito, sin importar de
# capytown_granprix -- sim_local es standalone, ver geometry_utils.py
# para el equivalente que usa el paquete ROS2 real).
# ----------------------------------------------------------------------
def normalizar_angulo(angulo: float) -> float:
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo <= -math.pi:
        angulo += 2.0 * math.pi
    return angulo


def diferencia_angular(objetivo: float, actual: float) -> float:
    return normalizar_angulo(objetivo - actual)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ----------------------------------------------------------------------
# Configuracion
# ----------------------------------------------------------------------
@dataclass
class UniqueLineConfig:
    """Un solo booleano decide el lado (``follow_right``/``follow_left``,
    mutuamente excluyentes). Todo lo demas (signo de control, signo de
    giro, ventanas angulares del LiDAR) se deriva en ``__post_init__``
    para no duplicar el if/else en cada lugar que use la config."""

    follow_right: bool = True
    follow_left: bool = False

    target_wall_dist: float = 0.12
    emergency_stop_dist: float = 0.10
    front_blocked_dist: float = 0.36
    front_clear_dist: float = 0.44
    lost_wall_dist: float = 0.34
    reacquire_wall_dist: float = 0.28
    collision_radius: float = 0.075
    safety_side_dist: float = 0.092

    Kp_wall: float = 1.35
    Kp_heading: float = 1.25
    deadband_dist: float = 0.024
    filter_alpha: float = 0.24
    w_limit: float = 0.75

    v_nom: float = 0.19
    v_align: float = 0.090
    v_corner: float = 0.065
    w_corner: float = 0.55
    v_clear: float = 0.095
    exterior_clear_dist: float = 0.18

    lost_required: int = 4
    clear_required: int = 4
    stable_required: int = 6
    blocked_required: int = 2

    # Tolerancias de la FSM (7 grados de alineacion tras giro, 10 grados
    # + 4 cm de estabilidad en CORNER_ALIGN -- ver secciones 7-10 del
    # pedido original).
    align_tolerance_deg: float = 7.0
    stable_heading_tolerance_deg: float = 10.0
    stable_dist_tolerance_m: float = 0.04

    # LiDAR: rango util y tolerancia angular para emparejar cada angulo
    # pedido (front_window/wall_window/wide_wall_window) con el rayo
    # mas cercano del scan real.
    range_min_m: float = 0.03
    range_max_m: float = 4.0
    sector_tol_deg: float = 1.0

    # Angostada de +-12 a +-8 grados: en el robot real, +-12 alcanzaba a
    # ver la pared lateral seguida (a target_wall_dist=0.12m) como si
    # fuera un obstaculo al frente, disparando giros falsos. Validado de
    # nuevo en sim_local tras angostar (10/10 SUCCESS sin cambios).
    front_window_deg: Tuple[float, ...] = (-8.0, -4.0, 0.0, 4.0, 8.0)

    # Derivados en __post_init__ (no pasar a mano):
    wall_side: str = field(init=False, default='RIGHT')
    wall_control_sign: float = field(init=False, default=1.0)
    turn_away_sign: float = field(init=False, default=1.0)
    turn_toward_sign: float = field(init=False, default=-1.0)
    wall_window_deg: Tuple[float, ...] = field(init=False, default=())
    wide_wall_window_deg: Tuple[float, ...] = field(init=False, default=())

    def __post_init__(self):
        assert self.follow_right != self.follow_left, (
            'follow_right y follow_left son mutuamente excluyentes: '
            'exactamente uno debe ser True.'
        )
        if self.follow_right:
            self.wall_side = 'RIGHT'
            self.wall_control_sign = 1.0
            self.turn_away_sign = 1.0     # INTERIOR_TURN_90: +w_corner (izquierda)
            self.turn_toward_sign = -1.0  # EXTERIOR_TURN_90: -w_corner (derecha)
            self.wall_window_deg = (-102.0, -96.0, -90.0, -84.0, -78.0)
            self.wide_wall_window_deg = (-140.0, -120.0, -100.0, -90.0, -80.0, -60.0, -50.0)
        else:
            self.wall_side = 'LEFT'
            self.wall_control_sign = -1.0
            self.turn_away_sign = -1.0    # INTERIOR_TURN_90: -w_corner (derecha)
            self.turn_toward_sign = 1.0   # EXTERIOR_TURN_90: +w_corner (izquierda)
            self.wall_window_deg = (78.0, 84.0, 90.0, 96.0, 102.0)
            self.wide_wall_window_deg = (50.0, 60.0, 80.0, 90.0, 100.0, 120.0, 140.0)


# ----------------------------------------------------------------------
# Extraccion de sectores del LiDAR (angulos discretos, no ventana
# continua -- se busca el rayo mas cercano a cada angulo pedido).
# ----------------------------------------------------------------------
def _angle_diff_arr(angulos: np.ndarray, objetivo: float) -> np.ndarray:
    d = angulos - objetivo
    return np.mod(d + math.pi, 2.0 * math.pi) - math.pi


def sector_samples(
    angulos_robot: np.ndarray,
    rangos: np.ndarray,
    angles_deg: Sequence[float],
    tol_deg: float,
    range_min: float,
    range_max: float,
) -> List[float]:
    """Para cada angulo en ``angles_deg``, toma la lectura del rayo mas
    cercano (dentro de ``tol_deg``) si es finita y esta en rango.
    Retorna la lista de lecturas validas encontradas (puede quedar
    vacia si ningun angulo tuvo un rayo valido cerca)."""
    vals = []
    tol = math.radians(tol_deg)
    for ad in angles_deg:
        objetivo = math.radians(ad)
        diffs = np.abs(_angle_diff_arr(angulos_robot, objetivo))
        idx = int(np.argmin(diffs))
        if diffs[idx] <= tol and np.isfinite(rangos[idx]) and range_min <= rangos[idx] <= range_max:
            vals.append(float(rangos[idx]))
    return vals


def compute_readings(
    angulos_robot: np.ndarray, rangos: np.ndarray, cfg: UniqueLineConfig
) -> Tuple[float, float, float]:
    """Retorna (front_dist, wall_dist_raw, side_min).

    Sin puntos validos en un sector, se usa ``range_max_m`` como
    centinela ("lejos"/"libre"/"pared perdida") -- no se usan flags
    booleanos de "valido" en esta logica, a diferencia del resto del
    proyecto: front_dist/wall_dist/side_min son simplemente distancias,
    y "lejos" ya significa lo mismo que "no hay pared ahi"."""
    front_vals = sector_samples(
        angulos_robot, rangos, cfg.front_window_deg, cfg.sector_tol_deg,
        cfg.range_min_m, cfg.range_max_m,
    )
    wall_vals = sector_samples(
        angulos_robot, rangos, cfg.wall_window_deg, cfg.sector_tol_deg,
        cfg.range_min_m, cfg.range_max_m,
    )
    side_vals = sector_samples(
        angulos_robot, rangos, cfg.wide_wall_window_deg, cfg.sector_tol_deg,
        cfg.range_min_m, cfg.range_max_m,
    )

    front_dist = min(front_vals) if front_vals else cfg.range_max_m
    wall_dist_raw = float(np.median(wall_vals)) if wall_vals else cfg.range_max_m
    side_min = min(side_vals) if side_vals else cfg.range_max_m
    return front_dist, wall_dist_raw, side_min


# ----------------------------------------------------------------------
# Maquina de estados
# ----------------------------------------------------------------------
class UniqueLineFSM:
    """Estado interno + logica de decision. ``step()`` se llama una vez
    por ciclo de control con las lecturas ya extraidas (``front_dist``,
    ``wall_dist_raw``, ``side_min``) y la pose actual, y retorna
    ``(linear_x, angular_z)``.
    """

    def __init__(self, cfg: UniqueLineConfig, heading_inicial: float = 0.0):
        self.cfg = cfg
        self.state = 'FOLLOW_WALL'
        self.target_heading = heading_inicial
        self.desired_heading = heading_inicial
        self.wall_dist_f: Optional[float] = None

        self.blocked_count = 0
        self.lost_count = 0
        self.align_count = 0
        self.stable_count = 0
        self.clear_count = 0  # usado solo por EMERGENCY_STOP

        self._pre_emergency_state = 'FOLLOW_WALL'
        self._clear_start_xy: Optional[Tuple[float, float]] = None

        # Lecturas del ciclo actual (fijadas al entrar a step()).
        self._x = 0.0
        self._y = 0.0
        self._yaw = heading_inicial
        self._front_dist = cfg.range_max_m
        self._side_min = cfg.range_max_m
        self._dt = 0.0

        self._STATE_HANDLERS = {
            'FOLLOW_WALL': self._handle_follow_wall,
            'INTERIOR_TURN_90': self._handle_interior_turn,
            'EXTERIOR_CLEAR': self._handle_exterior_clear,
            'EXTERIOR_TURN_90': self._handle_exterior_turn,
            'CORNER_ALIGN': self._handle_corner_align,
            'EMERGENCY_STOP': self._handle_emergency_stop,
        }

    # ------------------------------------------------------------
    def step(
        self, x: float, y: float, yaw: float,
        front_dist: float, wall_dist_raw: float, side_min: float, dt: float,
    ) -> Tuple[float, float]:
        self._x, self._y, self._yaw, self._dt = x, y, yaw, dt
        self._front_dist = front_dist
        self._side_min = side_min
        self._update_wall_filter(wall_dist_raw)

        self._check_emergency_entry()

        v, w = 0.0, 0.0
        for _ in range(4):  # deja encadenar transiciones dentro del mismo ciclo
            prev_state = self.state
            v, w = self._STATE_HANDLERS[self.state]()
            if self.state == prev_state:
                break
        return v, w

    # ------------------------------------------------------------
    # Internos
    # ------------------------------------------------------------
    def _update_wall_filter(self, raw: float) -> None:
        if self.wall_dist_f is None:
            self.wall_dist_f = raw
        else:
            a = self.cfg.filter_alpha
            self.wall_dist_f = (1.0 - a) * self.wall_dist_f + a * raw

    def _transition(self, new_state: str) -> None:
        self.state = new_state
        self.blocked_count = 0
        self.lost_count = 0
        self.align_count = 0
        self.stable_count = 0
        if new_state == 'EXTERIOR_CLEAR':
            self._clear_start_xy = (self._x, self._y)

    def _wall_control(self) -> float:
        cfg = self.cfg
        error = cfg.target_wall_dist - self.wall_dist_f
        if abs(error) < cfg.deadband_dist:
            error = 0.0
        w = (
            cfg.wall_control_sign * cfg.Kp_wall * error
            + cfg.Kp_heading * diferencia_angular(self.target_heading, self._yaw)
        )
        return _clamp(w, -cfg.w_limit, cfg.w_limit)

    def _heading_hold(self) -> float:
        cfg = self.cfg
        w = cfg.Kp_heading * diferencia_angular(self.target_heading, self._yaw)
        return _clamp(w, -cfg.w_limit, cfg.w_limit)

    def _apply_lateral_safety(self, v: float, w: float) -> Tuple[float, float]:
        cfg = self.cfg
        if self._side_min < cfg.safety_side_dist:
            v = min(v, 0.040)
            if cfg.follow_right:
                w = max(w, 0.24)   # alejarse de la pared derecha
            else:
                w = min(w, -0.24)  # alejarse de la pared izquierda
        return v, w

    # ------------------------------------------------------------
    # EMERGENCY_STOP: prioridad maxima, se evalua antes de despachar
    # el resto de estados (ver docstring del modulo).
    # ------------------------------------------------------------
    def _check_emergency_entry(self) -> None:
        if self.state != 'EMERGENCY_STOP' and self._front_dist < self.cfg.emergency_stop_dist:
            self._pre_emergency_state = self.state
            self.state = 'EMERGENCY_STOP'
            self.clear_count = 0

    def _handle_emergency_stop(self) -> Tuple[float, float]:
        cfg = self.cfg
        if self._front_dist > cfg.front_clear_dist:
            self.clear_count += 1
        else:
            self.clear_count = 0

        if self.clear_count >= cfg.clear_required:
            self.state = self._pre_emergency_state
            self.clear_count = 0
            # se deja que step() vuelva a despachar el estado restaurado
            # en la misma llamada (cascada de transiciones).

        return 0.0, 0.0

    # ------------------------------------------------------------
    # FOLLOW_WALL
    # ------------------------------------------------------------
    def _handle_follow_wall(self) -> Tuple[float, float]:
        cfg = self.cfg
        front = self._front_dist

        if front < cfg.front_blocked_dist:
            self.blocked_count += 1
        else:
            self.blocked_count = 0

        if self.blocked_count >= cfg.blocked_required:
            self.desired_heading = normalizar_angulo(
                self.target_heading + cfg.turn_away_sign * HALF_PI
            )
            self._transition('INTERIOR_TURN_90')
            return 0.0, 0.0

        if self.wall_dist_f > cfg.lost_wall_dist and front > cfg.front_clear_dist:
            self.lost_count += 1
            if self.lost_count >= cfg.lost_required:
                self.desired_heading = normalizar_angulo(
                    self.target_heading + cfg.turn_toward_sign * HALF_PI
                )
                self._transition('EXTERIOR_CLEAR')
                return 0.0, 0.0
        else:
            self.lost_count = 0

        v = min(cfg.v_nom, max(0.055, 0.52 * front))
        w = self._wall_control()
        v, w = self._apply_lateral_safety(v, w)
        return v, w

    # ------------------------------------------------------------
    # INTERIOR_TURN_90: gira ALEJANDOSE de la pared seguida.
    # ------------------------------------------------------------
    def _handle_interior_turn(self) -> Tuple[float, float]:
        cfg = self.cfg
        tol = math.radians(cfg.align_tolerance_deg)

        w = cfg.turn_away_sign * cfg.w_corner
        v = cfg.v_corner
        if self._front_dist < cfg.emergency_stop_dist or self._side_min < cfg.safety_side_dist:
            v = 0.0

        err = diferencia_angular(self._yaw, self.desired_heading)
        if abs(err) < tol:
            self.align_count += 1
            if self.align_count >= cfg.clear_required:
                self.target_heading = self.desired_heading
                self._transition('CORNER_ALIGN')
                return 0.0, 0.0
        else:
            self.align_count = 0

        return v, w

    # ------------------------------------------------------------
    # EXTERIOR_CLEAR: avanza recto un tramo corto antes de girar hacia
    # la pared, para no cortar la esquina exterior en diagonal.
    # ------------------------------------------------------------
    def _handle_exterior_clear(self) -> Tuple[float, float]:
        cfg = self.cfg
        if self._clear_start_xy is None:
            self._clear_start_xy = (self._x, self._y)

        dx = self._x - self._clear_start_xy[0]
        dy = self._y - self._clear_start_xy[1]
        avanzado = math.hypot(dx, dy)

        if avanzado >= cfg.exterior_clear_dist:
            self._transition('EXTERIOR_TURN_90')
            return 0.0, 0.0

        v = cfg.v_clear
        w = self._heading_hold()
        return v, w

    # ------------------------------------------------------------
    # EXTERIOR_TURN_90: gira HACIA la pared seguida.
    # ------------------------------------------------------------
    def _handle_exterior_turn(self) -> Tuple[float, float]:
        cfg = self.cfg
        tol = math.radians(cfg.align_tolerance_deg)

        w = cfg.turn_toward_sign * cfg.w_corner
        v = cfg.v_corner
        if self._side_min < cfg.safety_side_dist:
            v = 0.0

        err = diferencia_angular(self._yaw, self.desired_heading)
        if abs(err) < tol:
            self.align_count += 1
            if self.align_count >= cfg.clear_required:
                self.target_heading = self.desired_heading
                self._transition('CORNER_ALIGN')
                return 0.0, 0.0
        else:
            self.align_count = 0

        return v, w

    # ------------------------------------------------------------
    # CORNER_ALIGN: estabiliza contra la pared real tras cualquier giro.
    # ------------------------------------------------------------
    def _handle_corner_align(self) -> Tuple[float, float]:
        cfg = self.cfg
        front = self._front_dist

        if front < cfg.front_blocked_dist:
            self.blocked_count += 1
        else:
            self.blocked_count = 0

        if self.blocked_count >= cfg.blocked_required:
            self.desired_heading = normalizar_angulo(
                self.target_heading + cfg.turn_away_sign * HALF_PI
            )
            self._transition('INTERIOR_TURN_90')
            return 0.0, 0.0

        if self.wall_dist_f < cfg.reacquire_wall_dist:
            v = cfg.v_align
            w = self._wall_control()

            dist_ok = abs(self.wall_dist_f - cfg.target_wall_dist) < cfg.stable_dist_tolerance_m
            heading_ok = abs(diferencia_angular(self._yaw, self.target_heading)) < math.radians(
                cfg.stable_heading_tolerance_deg
            )
            if dist_ok and heading_ok:
                self.stable_count += 1
                if self.stable_count >= cfg.stable_required:
                    self._transition('FOLLOW_WALL')
                    return 0.0, 0.0
            else:
                self.stable_count = 0
        else:
            self.stable_count = 0
            v = cfg.v_align
            w = self._heading_hold()

            if self.wall_dist_f > cfg.lost_wall_dist and front > cfg.front_clear_dist:
                self.lost_count += 1
                if self.lost_count >= cfg.lost_required:
                    self.desired_heading = normalizar_angulo(
                        self.target_heading + cfg.turn_toward_sign * HALF_PI
                    )
                    self._transition('EXTERIOR_CLEAR')
                    return 0.0, 0.0
            else:
                self.lost_count = 0

        v, w = self._apply_lateral_safety(v, w)
        return v, w
