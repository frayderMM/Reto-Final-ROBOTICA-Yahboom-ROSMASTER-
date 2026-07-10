#!/usr/bin/env python3
"""Nodo ROS2 de la logica ``unique_line``: seguimiento de UNA sola
pared lateral (derecha o izquierda) con LiDAR, para chasis tipo
Ackermann (no rota sobre su propio eje).

Portado de ``sim_local/unique_line_control.py`` (misma FSM, mismos
nombres de parametros, sin cambios de logica) tras validarse en
``sim_local/unique_line_simulator.py``: 10/10 SUCCESS en las 10
pruebas pedidas (distancia lateral inicial 0.08-0.32 m, ruido leve en
las ultimas 4), 0 colisiones, angulo final ~88 grados -- ver
``sim_local/unique_line_report.md``. Solo la capa de entrada/salida es
especifica de ROS2 (LaserScan/Odometry -> Twist); la maquina de
estados es identica a la del simulador.

Suscribe:
    /scan       (sensor_msgs/LaserScan)
    /odom_raw   (nav_msgs/Odometry) -- yaw y posicion x,y (esta ultima
                solo para medir la distancia recorrida en EXTERIOR_CLEAR)

Publica:
    /cmd_vel    (geometry_msgs/Twist)

No depende de ``lidar_processor_node``/``LidarZones``: lee ``/scan``
directo y extrae sus propios sectores angulares (mas simple y
autonomo, pensado para poder correr solo, sin el resto del paquete
``capytown_granprix``). Reusa la calibracion de montaje del LiDAR
(``front_offset_deg``, ``invert_left_right``) ya validada para este
robot en ``granprix_params.yaml`` -- mismo LiDAR fisico, mismo offset.

Regla de diseno clave (evita "girar hasta que el frente este libre",
que corta las esquinas en diagonal): los giros de pared "hacia afuera"
(la pared se aleja/curva) se siguen con el control continuo de
angulo+distancia, sin necesitar un estado de giro aparte -- solo los
giros "hacia adentro" (pared que corta el paso al frente) o donde la
pared desaparece del todo (esquina exterior real) usan un giro
discreto de 90 grados, con una fase de alineacion despues (ver
``UniqueLineFSM`` mas abajo, identica a la del simulador).
"""

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from capytown_granprix.geometry_utils import angle_diff, normalize_angle, yaw_from_quaternion
from capytown_granprix.lidar_utils import compute_robot_frame_angles

HALF_PI = math.pi / 2.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ----------------------------------------------------------------------
# Configuracion (identica a sim_local/unique_line_control.py::UniqueLineConfig,
# ver ese modulo para el detalle de cada parametro).
# ----------------------------------------------------------------------
@dataclass
class UniqueLineConfig:
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

    align_tolerance_deg: float = 7.0
    stable_heading_tolerance_deg: float = 10.0
    stable_dist_tolerance_m: float = 0.04

    range_min_m: float = 0.03
    range_max_m: float = 4.0
    sector_tol_deg: float = 1.0

    # Angostada de +-12 a +-8 grados: en el robot real, +-12 alcanzaba a
    # ver la pared lateral seguida (a target_wall_dist=0.12m) como si
    # fuera un obstaculo al frente, disparando giros falsos.
    front_window_deg: Tuple[float, ...] = (-8.0, -4.0, 0.0, 4.0, 8.0)

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
            self.turn_away_sign = 1.0
            self.turn_toward_sign = -1.0
            self.wall_window_deg = (-102.0, -96.0, -90.0, -84.0, -78.0)
            self.wide_wall_window_deg = (-140.0, -120.0, -100.0, -90.0, -80.0, -60.0, -50.0)
        else:
            self.wall_side = 'LEFT'
            self.wall_control_sign = -1.0
            self.turn_away_sign = -1.0
            self.turn_toward_sign = 1.0
            self.wall_window_deg = (78.0, 84.0, 90.0, 96.0, 102.0)
            self.wide_wall_window_deg = (50.0, 60.0, 80.0, 90.0, 100.0, 120.0, 140.0)


def _angle_diff_arr(angulos: np.ndarray, objetivo: float) -> np.ndarray:
    d = angulos - objetivo
    return np.mod(d + math.pi, 2.0 * math.pi) - math.pi


def sector_samples(
    angulos_robot: np.ndarray, rangos: np.ndarray, angles_deg: Sequence[float],
    tol_deg: float, range_min: float, range_max: float,
) -> List[float]:
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
# Maquina de estados (identica a sim_local/unique_line_control.py::UniqueLineFSM).
# ----------------------------------------------------------------------
class UniqueLineFSM:

    def __init__(self, cfg: UniqueLineConfig, heading_inicial: float = 0.0):
        self.cfg = cfg
        self.state = 'FOLLOW_WALL'
        self.target_heading = heading_inicial
        self.desired_heading = heading_inicial
        self.wall_dist_f = None

        self.blocked_count = 0
        self.lost_count = 0
        self.align_count = 0
        self.stable_count = 0
        self.clear_count = 0

        self._pre_emergency_state = 'FOLLOW_WALL'
        self._clear_start_xy = None

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

    def step(self, x, y, yaw, front_dist, wall_dist_raw, side_min, dt):
        self._x, self._y, self._yaw, self._dt = x, y, yaw, dt
        self._front_dist = front_dist
        self._side_min = side_min
        self._update_wall_filter(wall_dist_raw)

        self._check_emergency_entry()

        # Un solo despacho por ciclo externo -- encadenar transiciones
        # dentro del mismo ciclo dejaba que un estado recien entrado
        # reevaluara la MISMA lectura ya usada por el estado anterior
        # para decidir la transicion (encontrado en pista real: le
        # "regalaba" a CORNER_ALIGN una confirmacion de frente-bloqueado
        # gratis justo al terminar un giro, disparando 2-3 giros de 90
        # grados seguidos en la misma esquina en vez de uno solo).
        v, w = self._STATE_HANDLERS[self.state]()
        return v, w

    def _update_wall_filter(self, raw):
        if self.wall_dist_f is None:
            self.wall_dist_f = raw
        else:
            a = self.cfg.filter_alpha
            self.wall_dist_f = (1.0 - a) * self.wall_dist_f + a * raw

    def _transition(self, new_state):
        self.state = new_state
        self.blocked_count = 0
        self.lost_count = 0
        self.align_count = 0
        self.stable_count = 0
        if new_state == 'EXTERIOR_CLEAR':
            self._clear_start_xy = (self._x, self._y)

    def _wall_control(self):
        cfg = self.cfg
        error = cfg.target_wall_dist - self.wall_dist_f
        if abs(error) < cfg.deadband_dist:
            error = 0.0
        w = (
            cfg.wall_control_sign * cfg.Kp_wall * error
            + cfg.Kp_heading * angle_diff(self.target_heading, self._yaw)
        )
        return _clamp(w, -cfg.w_limit, cfg.w_limit)

    def _heading_hold(self):
        cfg = self.cfg
        w = cfg.Kp_heading * angle_diff(self.target_heading, self._yaw)
        return _clamp(w, -cfg.w_limit, cfg.w_limit)

    def _apply_lateral_safety(self, v, w):
        cfg = self.cfg
        if self._side_min < cfg.safety_side_dist:
            v = min(v, 0.040)
            if cfg.follow_right:
                w = max(w, 0.24)
            else:
                w = min(w, -0.24)
        return v, w

    def _check_emergency_entry(self):
        if self.state != 'EMERGENCY_STOP' and self._front_dist < self.cfg.emergency_stop_dist:
            self._pre_emergency_state = self.state
            self.state = 'EMERGENCY_STOP'
            self.clear_count = 0

    def _handle_emergency_stop(self):
        cfg = self.cfg
        if self._front_dist > cfg.front_clear_dist:
            self.clear_count += 1
        else:
            self.clear_count = 0

        if self.clear_count >= cfg.clear_required:
            self.state = self._pre_emergency_state
            self.clear_count = 0

        return 0.0, 0.0

    def _handle_follow_wall(self):
        cfg = self.cfg
        front = self._front_dist

        if front < cfg.front_blocked_dist:
            self.blocked_count += 1
        else:
            self.blocked_count = 0

        if self.blocked_count >= cfg.blocked_required:
            self.desired_heading = normalize_angle(self.target_heading + cfg.turn_away_sign * HALF_PI)
            self._transition('INTERIOR_TURN_90')
            return 0.0, 0.0

        if self.wall_dist_f > cfg.lost_wall_dist and front > cfg.front_clear_dist:
            self.lost_count += 1
            if self.lost_count >= cfg.lost_required:
                self.desired_heading = normalize_angle(self.target_heading + cfg.turn_toward_sign * HALF_PI)
                self._transition('EXTERIOR_CLEAR')
                return 0.0, 0.0
        else:
            self.lost_count = 0

        v = min(cfg.v_nom, max(0.055, 0.52 * front))
        w = self._wall_control()
        v, w = self._apply_lateral_safety(v, w)
        return v, w

    def _handle_interior_turn(self):
        cfg = self.cfg
        tol = math.radians(cfg.align_tolerance_deg)

        w = cfg.turn_away_sign * cfg.w_corner
        v = cfg.v_corner
        if self._front_dist < cfg.emergency_stop_dist or self._side_min < cfg.safety_side_dist:
            v = 0.0

        err = angle_diff(self._yaw, self.desired_heading)
        if abs(err) < tol:
            self.align_count += 1
            if self.align_count >= cfg.clear_required:
                self.target_heading = self.desired_heading
                self._transition('CORNER_ALIGN')
                return 0.0, 0.0
        else:
            # Decrementar (no resetear a 0): girando rapido, la ventana
            # de tolerancia se cruza en pocos ciclos -- una sola lectura
            # ruidosa apenas afuera del rango no debe borrar todo el
            # progreso previo (encontrado en pista real: giro de 360+90
            # grados en vez de 90 por esto).
            self.align_count = max(0, self.align_count - 1)

        return v, w

    def _handle_exterior_clear(self):
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

    def _handle_exterior_turn(self):
        cfg = self.cfg
        tol = math.radians(cfg.align_tolerance_deg)

        w = cfg.turn_toward_sign * cfg.w_corner
        v = cfg.v_corner
        if self._side_min < cfg.safety_side_dist:
            v = 0.0

        err = angle_diff(self._yaw, self.desired_heading)
        if abs(err) < tol:
            self.align_count += 1
            if self.align_count >= cfg.clear_required:
                self.target_heading = self.desired_heading
                self._transition('CORNER_ALIGN')
                return 0.0, 0.0
        else:
            self.align_count = max(0, self.align_count - 1)

        return v, w

    def _handle_corner_align(self):
        cfg = self.cfg
        front = self._front_dist

        if front < cfg.front_blocked_dist:
            self.blocked_count += 1
        else:
            self.blocked_count = 0

        if self.blocked_count >= cfg.blocked_required:
            self.desired_heading = normalize_angle(self.target_heading + cfg.turn_away_sign * HALF_PI)
            self._transition('INTERIOR_TURN_90')
            return 0.0, 0.0

        if self.wall_dist_f < cfg.reacquire_wall_dist:
            v = cfg.v_align
            w = self._wall_control()

            dist_ok = abs(self.wall_dist_f - cfg.target_wall_dist) < cfg.stable_dist_tolerance_m
            heading_ok = abs(angle_diff(self._yaw, self.target_heading)) < math.radians(
                cfg.stable_heading_tolerance_deg
            )
            if dist_ok and heading_ok:
                self.stable_count += 1
                if self.stable_count >= cfg.stable_required:
                    self._transition('FOLLOW_WALL')
                    return 0.0, 0.0
            else:
                self.stable_count = max(0, self.stable_count - 1)
        else:
            self.stable_count = 0
            v = cfg.v_align
            w = self._heading_hold()

            if self.wall_dist_f > cfg.lost_wall_dist and front > cfg.front_clear_dist:
                self.lost_count += 1
                if self.lost_count >= cfg.lost_required:
                    self.desired_heading = normalize_angle(self.target_heading + cfg.turn_toward_sign * HALF_PI)
                    self._transition('EXTERIOR_CLEAR')
                    return 0.0, 0.0
            else:
                self.lost_count = 0

        v, w = self._apply_lateral_safety(v, w)
        return v, w


# ----------------------------------------------------------------------
# Nodo ROS2
# ----------------------------------------------------------------------
class UniqueLineNode(Node):

    def __init__(self):
        super().__init__('unique_line')
        self._declare_parameters()
        self._cfg = self._read_config()

        self._front_offset_rad = math.radians(float(self.get_parameter('front_offset_deg').value))
        self._sign = -1 if bool(self.get_parameter('invert_left_right').value) else 1
        self._control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self._scan_topic = self.get_parameter('scan_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self._scan_msg = None
        self._scan_ready = False
        self._odom_ready = False
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0

        self._fsm = None  # se crea al recibir la primera odometria (heading inicial)
        self._last_logged_state = None
        self._diag_log_period_s = float(self.get_parameter('diag_log_period_s').value)
        self._last_diag_log = self.get_clock().now()

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_timer(1.0 / self._control_rate_hz, self._on_timer)

        self.get_logger().info(
            f'unique_line listo: lado={self._cfg.wall_side} '
            f'objetivo={self._cfg.target_wall_dist * 100:.1f}cm '
            f'v_nom={self._cfg.v_nom:.2f}m/s'
        )

    # ------------------------------------------------------------
    def _declare_parameters(self):
        defaults = {
            'scan_topic': '/scan',
            'odom_topic': '/odom_raw',
            'cmd_vel_topic': '/cmd_vel',
            'control_rate_hz': 20.0,

            # Calibracion de montaje del LiDAR -- mismos valores que
            # lidar_processor_node para este robot (ver granprix_params.yaml).
            'front_offset_deg': 180.0,
            'invert_left_right': False,

            'follow_right': True,
            'follow_left': False,

            'target_wall_dist': 0.12,
            'emergency_stop_dist': 0.10,
            'front_blocked_dist': 0.36,
            'front_clear_dist': 0.44,
            'lost_wall_dist': 0.34,
            'reacquire_wall_dist': 0.28,
            'collision_radius': 0.075,
            'safety_side_dist': 0.092,

            'Kp_wall': 1.35,
            'Kp_heading': 1.25,
            'deadband_dist': 0.024,
            'filter_alpha': 0.24,
            'w_limit': 0.75,

            'v_nom': 0.19,
            'v_align': 0.090,
            'v_corner': 0.065,
            'w_corner': 0.55,
            'v_clear': 0.095,
            'exterior_clear_dist': 0.18,

            'lost_required': 4,
            'clear_required': 4,
            'stable_required': 6,
            'blocked_required': 2,

            'align_tolerance_deg': 7.0,
            'stable_heading_tolerance_deg': 10.0,
            'stable_dist_tolerance_m': 0.04,

            'range_min_m': 0.03,
            'range_max_m': 4.0,
            'sector_tol_deg': 1.0,

            # Angulos discretos (grados) del cono frontal -- angostar
            # esto si la pared lateral seguida se confunde con un
            # obstaculo al frente (ver README_unique_line.md).
            'front_window_deg': [-8.0, -4.0, 0.0, 4.0, 8.0],

            # Diagnostico: log periodico (segundos) con estado y
            # distancias (front/wall_dist_f/side_min) ademas del log de
            # cada cambio de estado (ese siempre se imprime). 0 = solo
            # log de transiciones, sin el periodico. Util para pegar la
            # salida de terminal en UNIQUE_LINE_DEBUG.md al diagnosticar
            # un problema en pista (ver ese archivo).
            'diag_log_period_s': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_config(self) -> UniqueLineConfig:
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        return UniqueLineConfig(
            follow_right=bool(g('follow_right')),
            follow_left=bool(g('follow_left')),
            target_wall_dist=float(g('target_wall_dist')),
            emergency_stop_dist=float(g('emergency_stop_dist')),
            front_blocked_dist=float(g('front_blocked_dist')),
            front_clear_dist=float(g('front_clear_dist')),
            lost_wall_dist=float(g('lost_wall_dist')),
            reacquire_wall_dist=float(g('reacquire_wall_dist')),
            collision_radius=float(g('collision_radius')),
            safety_side_dist=float(g('safety_side_dist')),
            Kp_wall=float(g('Kp_wall')),
            Kp_heading=float(g('Kp_heading')),
            deadband_dist=float(g('deadband_dist')),
            filter_alpha=float(g('filter_alpha')),
            w_limit=float(g('w_limit')),
            v_nom=float(g('v_nom')),
            v_align=float(g('v_align')),
            v_corner=float(g('v_corner')),
            w_corner=float(g('w_corner')),
            v_clear=float(g('v_clear')),
            exterior_clear_dist=float(g('exterior_clear_dist')),
            lost_required=int(g('lost_required')),
            clear_required=int(g('clear_required')),
            stable_required=int(g('stable_required')),
            blocked_required=int(g('blocked_required')),
            align_tolerance_deg=float(g('align_tolerance_deg')),
            stable_heading_tolerance_deg=float(g('stable_heading_tolerance_deg')),
            stable_dist_tolerance_m=float(g('stable_dist_tolerance_m')),
            range_min_m=float(g('range_min_m')),
            range_max_m=float(g('range_max_m')),
            sector_tol_deg=float(g('sector_tol_deg')),
            front_window_deg=tuple(float(a) for a in g('front_window_deg')),
        )

    # ------------------------------------------------------------
    # Callbacks de suscripcion (solo cachean el ultimo mensaje; el
    # ciclo de control real corre en _on_timer a tasa fija).
    # ------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_msg = msg
        self._scan_ready = True

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self._odom_ready:
            # Heading inicial = rumbo real del robot al arrancar (no se
            # asume 0/frente-mundo): el modulo es autonomo, no sabe en
            # que direccion del mundo esta la pista.
            self._fsm = UniqueLineFSM(self._cfg, heading_inicial=self._yaw)
            self.get_logger().info(
                f'heading inicial capturado: {math.degrees(self._yaw):.1f} deg'
            )
        self._odom_ready = True

    # ------------------------------------------------------------
    def _on_timer(self) -> None:
        if not (self._scan_ready and self._odom_ready and self._fsm is not None):
            return

        msg = self._scan_msg
        ranges = np.asarray(msg.ranges, dtype=float)
        robot_angles = compute_robot_frame_angles(
            ranges, msg.angle_min, msg.angle_increment, self._front_offset_rad, self._sign
        )
        front_dist, wall_dist_raw, side_min = compute_readings(robot_angles, ranges, self._cfg)

        dt = 1.0 / self._control_rate_hz
        v, w = self._fsm.step(
            self._odom_x, self._odom_y, self._yaw, front_dist, wall_dist_raw, side_min, dt
        )

        self._log_diagnostico(front_dist, wall_dist_raw, v, w)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self._cmd_pub.publish(cmd)

    def _log_diagnostico(self, front_dist: float, wall_dist_raw: float, v: float, w: float) -> None:
        """Log de diagnostico: SIEMPRE en cada cambio de estado, y ademas
        cada ``diag_log_period_s`` segundos aunque no cambie de estado
        (para ver si una lectura oscila o queda mal calibrada sin llegar
        a disparar una transicion). Pensado para pegar la salida de
        terminal en UNIQUE_LINE_DEBUG.md al reportar un problema."""
        fsm = self._fsm
        state = fsm.state

        if state != self._last_logged_state:
            self.get_logger().info(
                f'ESTADO: {self._last_logged_state} -> {state} | '
                f'front={front_dist:.3f}m wall_raw={wall_dist_raw:.3f}m '
                f'wall_f={fsm.wall_dist_f:.3f}m side_min={fsm._side_min:.3f}m '
                f'yaw={math.degrees(self._yaw):.1f}deg '
                f'target_heading={math.degrees(fsm.target_heading):.1f}deg '
                f'v={v:.3f} w={w:.3f}'
            )
            self._last_logged_state = state
            self._last_diag_log = self.get_clock().now()
            return

        if self._diag_log_period_s <= 0.0:
            return
        elapsed = (self.get_clock().now() - self._last_diag_log).nanoseconds / 1e9
        if elapsed >= self._diag_log_period_s:
            self.get_logger().info(
                f'diag: estado={state} front={front_dist:.3f}m wall_raw={wall_dist_raw:.3f}m '
                f'wall_f={fsm.wall_dist_f:.3f}m side_min={fsm._side_min:.3f}m '
                f'yaw={math.degrees(self._yaw):.1f}deg v={v:.3f} w={w:.3f}'
            )
            self._last_diag_log = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = UniqueLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
