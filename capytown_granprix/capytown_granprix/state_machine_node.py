#!/usr/bin/env python3
"""Nodo de decision de intersecciones y maquina de estados principal.

Es el UNICO nodo que escribe en ``/cmd_vel``: centraliza toda decision
de movimiento para evitar que dos publicadores manden comandos
contradictorios al mismo tiempo. Mientras el estado es
AVANZAR_PARALELO, reenvia la sugerencia de ``wall_follower_node``
(``/wall_follow/cmd_vel_suggestion``); en el resto de estados calcula
sus propios comandos (girar detenido-lento, alinear, detener).

Maquina de estados (ver logica_pared_derecha_robot.md y
DETALLE RETO 3.md):

    INICIAR -> AVANZAR_PARALELO -> DETECTAR_CRUCE -> BUSCAR_PARE
    -> DECIDIR -> GIRAR -> ALINEAR -> VERIFICAR_META -> (META
    o vuelve a AVANZAR_PARALELO)

Se agrega un estado adicional ``DETENIDO`` (fuera de la lista pedida)
solo como red de seguridad ante un limite de celdas recorridas sin
llegar a la meta (evita loops infinitos por fallas de sensor); no
reemplaza ni altera el flujo principal solicitado.

Nota sobre giros con chasis Ackermann: un vehiculo con direccion
Ackermann no puede rotar sobre su propio eje (radio de giro cero). El
estado GIRAR aproxima el "giro detenido" del documento de referencia
con un arco de avance lento y radio de giro pequeno (velocidad lineal
baja + angular maxima), usando el yaw de ``/odom_raw`` como
referencia de cierre en vez de tiempo fijo. Esto se debe calibrar en
pista (ver README).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from capytown_interfaces.msg import LidarZones, RobotEvent
from capytown_granprix import event_types as EV
from capytown_granprix.geometry_utils import angle_diff, normalize_angle, yaw_from_quaternion
from capytown_granprix.grid_map import GridTracker


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine')
        self._declare_parameters()
        self._read_parameters()

        self._grid = GridTracker.from_cell_name(self._celda_inicio, self._heading_inicial)

        # Estado de la maquina
        self._state = 'INICIAR'
        self._terminado = False

        # Datos de sensores (ultimo valor recibido)
        self._zones = None
        self._zones_ready = False
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0
        self._odom_ready = False
        self._pare_activo = False
        self._wall_follow_cmd = Twist()

        # Variables de trabajo por estado
        self._cell_start_xy = (0.0, 0.0)
        self._num_celdas = 0
        self._cruce_muestras = None
        self._derecha_libre = False
        self._frente_libre = False
        self._izquierda_libre = False
        self._buscar_pare_start = None
        self._pare_hold_start = None
        self._celdas_pare_respetadas = set()
        self._decision_actual = 'NINGUNO'
        self._giro_objetivo = 0.0
        self._alinear_start = None

        self._esperando_obstaculo = False
        self._espera_obstaculo_inicio = None

        self._STATE_HANDLERS = {
            'INICIAR': self._handle_iniciar,
            'AVANZAR_PARALELO': self._handle_avanzar_paralelo,
            'DETECTAR_CRUCE': self._handle_detectar_cruce,
            'BUSCAR_PARE': self._handle_buscar_pare,
            'DECIDIR': self._handle_decidir,
            'GIRAR': self._handle_girar,
            'ALINEAR': self._handle_alinear,
            'VERIFICAR_META': self._handle_verificar_meta,
            'META': self._handle_meta,
            'DETENIDO': self._handle_detenido,
        }

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._event_pub = self.create_publisher(RobotEvent, self._event_topic, 10)
        self._state_pub = self.create_publisher(String, self._robot_state_topic, 10)

        self.create_subscription(
            LidarZones, self._lidar_zones_topic, self._on_zones, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, self._pare_topic, self._on_pare, 10)
        self.create_subscription(Twist, self._wall_follow_topic, self._on_wall_follow, 10)

        self.create_timer(1.0 / self._control_rate_hz, self._on_timer)

        self.get_logger().info(
            f'state_machine listo: inicio={self._celda_inicio} meta={self._celda_meta} '
            f'heading_inicial={self._heading_inicial}'
        )

    # ------------------------------------------------------------------
    # Parametros
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        defaults = {
            'lidar_zones_topic': '/lidar_zones',
            'odom_topic': '/odom_raw',
            'cmd_vel_topic': '/cmd_vel',
            'wall_follow_topic': '/wall_follow/cmd_vel_suggestion',
            'pare_topic': '/pare_detectado',
            'event_topic': '/robot_event',
            'robot_state_topic': '/robot_state',
            'usar_camara': True,
            'control_rate_hz': 20.0,
            'umbral_frente_pared_m': 0.25,
            'umbral_frente_libre_m': 0.35,
            'umbral_lado_libre_m': 0.40,
            # Regla general de seguridad (siempre activa, en cualquier
            # estado): objeto al frente mas cerca que esto -> detenerse
            # de inmediato, esperar y volver a preguntar si esta libre.
            'umbral_colision_m': 0.10,
            'tiempo_espera_obstaculo_s': 2.0,
            'distancia_celda_m': 0.60,
            'margen_avance_m': 0.05,
            'muestras_confirmacion': 5,
            'consenso_minimo': 4,
            'velocidad_giro_lineal_mps': 0.08,
            'velocidad_giro_angular_radps': 0.5,
            'tolerancia_giro_deg': 4.0,
            'tolerancia_alineacion_m': 0.02,
            'tiempo_max_alinear_s': 4.0,
            'velocidad_alineacion_lineal_mps': 0.06,
            'velocidad_alineacion_angular_radps': 0.3,
            'tiempo_pare_s': 3.0,
            'tiempo_espera_camara_s': 0.5,
            'celda_inicio': 'A4',
            'celda_meta': 'F1',
            'heading_inicial': 'NORTE',
            'max_celdas_recorridas': 60,
            # Factores de correccion de escala del odometro (calibrados en
            # pista: avance real 76 cm / odometro 78.3 cm y giro real 90 /
            # odometro 90.92). Dejar en 1.0 si se recalibra desde cero.
            'factor_dist_odom': 0.9474,
            'factor_ang_odom': 0.9899,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self._lidar_zones_topic = g('lidar_zones_topic')
        self._odom_topic = g('odom_topic')
        self._cmd_vel_topic = g('cmd_vel_topic')
        self._wall_follow_topic = g('wall_follow_topic')
        self._pare_topic = g('pare_topic')
        self._event_topic = g('event_topic')
        self._robot_state_topic = g('robot_state_topic')

        self._usar_camara = bool(g('usar_camara'))
        self._control_rate_hz = float(g('control_rate_hz'))

        self._umbral_frente_pared = float(g('umbral_frente_pared_m'))
        self._umbral_frente_libre = float(g('umbral_frente_libre_m'))
        self._umbral_lado_libre = float(g('umbral_lado_libre_m'))
        self._umbral_colision = float(g('umbral_colision_m'))
        self._distancia_celda = float(g('distancia_celda_m'))
        self._margen_avance = float(g('margen_avance_m'))

        self._muestras_confirmacion = int(g('muestras_confirmacion'))
        self._consenso_minimo = int(g('consenso_minimo'))

        self._v_giro_lineal = float(g('velocidad_giro_lineal_mps'))
        self._v_giro_angular = float(g('velocidad_giro_angular_radps'))
        self._tolerancia_giro_rad = math.radians(float(g('tolerancia_giro_deg')))

        self._tolerancia_alineacion = float(g('tolerancia_alineacion_m'))
        self._tiempo_max_alinear = float(g('tiempo_max_alinear_s'))
        self._v_alinear_lineal = float(g('velocidad_alineacion_lineal_mps'))
        self._v_alinear_angular = float(g('velocidad_alineacion_angular_radps'))

        self._tiempo_pare = float(g('tiempo_pare_s'))
        self._tiempo_espera_camara = float(g('tiempo_espera_camara_s'))

        self._tiempo_espera_obstaculo = float(g('tiempo_espera_obstaculo_s'))

        self._celda_inicio = str(g('celda_inicio'))
        self._celda_meta = str(g('celda_meta'))
        self._heading_inicial = str(g('heading_inicial'))
        self._max_celdas = int(g('max_celdas_recorridas'))

        self._factor_dist_odom = float(g('factor_dist_odom'))
        self._factor_ang_odom = float(g('factor_ang_odom'))

    # ------------------------------------------------------------------
    # Callbacks de suscripcion
    # ------------------------------------------------------------------
    def _on_zones(self, msg: LidarZones):
        self._zones = msg
        self._zones_ready = True

    def _on_odom(self, msg: Odometry):
        # Correccion de escala del odometro (medida en pista, ver README):
        # el ROSMASTER R2 sobreestima tanto distancia como angulo girado,
        # de forma consistente, por lo que se corrige con un factor fijo.
        self._odom_x = msg.pose.pose.position.x * self._factor_dist_odom
        self._odom_y = msg.pose.pose.position.y * self._factor_dist_odom
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation) * self._factor_ang_odom
        self._odom_ready = True

    def _on_pare(self, msg: Bool):
        self._pare_activo = bool(msg.data)

    def _on_wall_follow(self, msg: Twist):
        self._wall_follow_cmd = msg

    # ------------------------------------------------------------------
    # Ciclo de control principal
    # ------------------------------------------------------------------
    def _on_timer(self):
        if not (self._odom_ready and self._zones_ready):
            return

        if self._handle_obstaculo_frente():
            return

        self._STATE_HANDLERS[self._state]()

    def _handle_obstaculo_frente(self) -> bool:
        """Regla general de seguridad, activa en cualquier estado.

        Si hay un objeto al frente mas cerca que ``umbral_colision_m``,
        detiene el robot de inmediato, espera ``tiempo_espera_obstaculo_s``
        y vuelve a comprobar si ya esta libre; si sigue bloqueado,
        reinicia la espera (queda preguntando en bucle hasta que se
        libere). Retorna True si este ciclo ya publico un comando (el
        llamador debe omitir el despacho normal de estados).
        """
        if self._terminado:
            return False

        z = self._zones
        frente_bloqueado = z.front_valid and z.front < self._umbral_colision

        if self._esperando_obstaculo:
            if frente_bloqueado:
                self._publish_twist(Twist())
                elapsed = (
                    self.get_clock().now() - self._espera_obstaculo_inicio
                ).nanoseconds / 1e9
                if elapsed >= self._tiempo_espera_obstaculo:
                    # Se cumplio la espera y sigue bloqueado: volver a
                    # preguntar en el proximo ciclo tras otra espera igual.
                    self._espera_obstaculo_inicio = self.get_clock().now()
                return True
            self._esperando_obstaculo = False
            return False

        if frente_bloqueado:
            self._publish_twist(Twist())
            self._publish_event(
                EV.COLISION, f'obstaculo a {z.front:.2f} m cerca de {self._grid.cell}'
            )
            self._esperando_obstaculo = True
            self._espera_obstaculo_inicio = self.get_clock().now()
            return True

        return False

    # ------------------------------------------------------------------
    # Estados
    # ------------------------------------------------------------------
    def _handle_iniciar(self):
        self._publish_event(
            EV.INICIO, f'inicio en {self._grid.cell}, heading {self._grid.heading}'
        )
        self._begin_avanzar_paralelo()
        self._set_state('AVANZAR_PARALELO')

    def _begin_avanzar_paralelo(self):
        self._cell_start_xy = (self._odom_x, self._odom_y)

    def _handle_avanzar_paralelo(self):
        dx = self._odom_x - self._cell_start_xy[0]
        dy = self._odom_y - self._cell_start_xy[1]
        avance = math.hypot(dx, dy)

        z = self._zones
        frente_cerca = z.front_valid and z.front < self._umbral_frente_pared

        if avance >= (self._distancia_celda - self._margen_avance) or frente_cerca:
            self._publish_twist(Twist())
            self._num_celdas += 1
            self._grid.advance_cell()
            self._publish_event(
                EV.CELDA_AVANZADA, f'celda {self._grid.cell} (#{self._num_celdas})'
            )

            if self._num_celdas > self._max_celdas:
                self._publish_event(
                    EV.TIMEOUT, 'limite de celdas recorridas alcanzado sin llegar a la meta'
                )
                self._terminado = True
                self._set_state('DETENIDO')
            else:
                self._set_state('DETECTAR_CRUCE')
            return

        self._publish_twist(self._wall_follow_cmd)

    def _handle_detectar_cruce(self):
        self._publish_twist(Twist())

        if self._cruce_muestras is None:
            self._cruce_muestras = {'right': [], 'front': [], 'left': []}

        z = self._zones
        self._cruce_muestras['right'].append(
            bool(z.right_valid and z.right > self._umbral_lado_libre)
        )
        self._cruce_muestras['front'].append(
            bool(z.front_valid and z.front > self._umbral_frente_libre)
        )
        self._cruce_muestras['left'].append(
            bool(z.left_valid and z.left > self._umbral_lado_libre)
        )

        if len(self._cruce_muestras['right']) < self._muestras_confirmacion:
            return

        def consenso(muestras):
            return sum(muestras) >= self._consenso_minimo

        self._derecha_libre = consenso(self._cruce_muestras['right'])
        self._frente_libre = consenso(self._cruce_muestras['front'])
        self._izquierda_libre = consenso(self._cruce_muestras['left'])
        self._cruce_muestras = None

        self._publish_event(
            EV.CRUCE,
            f'derecha={self._derecha_libre} frente={self._frente_libre} '
            f'izquierda={self._izquierda_libre}',
        )

        self._buscar_pare_start = self.get_clock().now()
        self._pare_hold_start = None
        self._set_state('BUSCAR_PARE')

    def _handle_buscar_pare(self):
        self._publish_twist(Twist())

        if not self._usar_camara:
            self._set_state('DECIDIR')
            return

        cell = self._grid.cell

        # Si ya se inicio el conteo de los 3 s, completarlo sin importar
        # parpadeos momentaneos de la deteccion (evita abortar el PARE
        # a mitad de camino si la camara pierde el color rojo un frame).
        if self._pare_hold_start is not None:
            elapsed = (self.get_clock().now() - self._pare_hold_start).nanoseconds / 1e9
            if elapsed >= self._tiempo_pare:
                self._celdas_pare_respetadas.add(cell)
                self._publish_event(EV.PARE_RESPETADO, f'PARE respetado en {cell}')
                self._set_state('DECIDIR')
            return

        if self._pare_activo and cell not in self._celdas_pare_respetadas:
            self._publish_event(EV.PARE_DETECTADO, f'senal PARE detectada en {cell}')
            self._pare_hold_start = self.get_clock().now()
            return

        elapsed_settle = (self.get_clock().now() - self._buscar_pare_start).nanoseconds / 1e9
        if elapsed_settle >= self._tiempo_espera_camara:
            self._set_state('DECIDIR')

    def _handle_decidir(self):
        if self._derecha_libre:
            direction = 'DERECHA'
        elif self._frente_libre:
            direction = 'NINGUNO'
        elif self._izquierda_libre:
            direction = 'IZQUIERDA'
        else:
            direction = 'ATRAS'
            self._publish_event(EV.DEAD_END, f'callejon sin salida en {self._grid.cell}')

        self._decision_actual = direction

        if direction == 'NINGUNO':
            self._alinear_start = None
            self._set_state('ALINEAR')
            return

        self._giro_objetivo = self._compute_turn_target(self._yaw, direction)
        self._publish_event(EV.GIRO, f'{direction} desde {self._grid.cell}')
        self._set_state('GIRAR')

    @staticmethod
    def _compute_turn_target(yaw: float, direction: str) -> float:
        if direction == 'DERECHA':
            delta = -math.pi / 2.0
        elif direction == 'IZQUIERDA':
            delta = math.pi / 2.0
        elif direction == 'ATRAS':
            delta = math.pi
        else:
            delta = 0.0
        return normalize_angle(yaw + delta)

    def _handle_girar(self):
        error = angle_diff(self._giro_objetivo, self._yaw)

        if abs(error) <= self._tolerancia_giro_rad:
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)
            self._alinear_start = None
            self._set_state('ALINEAR')
            return

        # Chasis Ackermann: no puede rotar en el sitio. Se aproxima el
        # giro con avance lento + direccion maxima, cerrando el lazo
        # con el yaw de la odometria (no con tiempo fijo).
        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        cmd.angular.z = self._v_giro_angular if error > 0.0 else -self._v_giro_angular
        self._publish_twist(cmd)

    def _handle_alinear(self):
        if self._alinear_start is None:
            self._alinear_start = self.get_clock().now()

        z = self._zones
        if not (z.right_front_valid and z.right_rear_valid):
            # Sin pared derecha de referencia (p.ej. abertura tras el
            # giro): el yaw de GIRAR ya dejo al robot orientado al
            # cardinal correcto, se continua sin correccion adicional.
            self._alinear_start = None
            self._set_state('VERIFICAR_META')
            return

        error_angulo = z.right_front - z.right_rear
        elapsed = (self.get_clock().now() - self._alinear_start).nanoseconds / 1e9

        if abs(error_angulo) <= self._tolerancia_alineacion or elapsed >= self._tiempo_max_alinear:
            self._publish_twist(Twist())
            self._alinear_start = None
            self._set_state('VERIFICAR_META')
            return

        cmd = Twist()
        cmd.linear.x = self._v_alinear_lineal
        cmd.angular.z = -self._v_alinear_angular if error_angulo > 0.0 else self._v_alinear_angular
        self._publish_twist(cmd)

    def _handle_verificar_meta(self):
        if self._grid.cell == self._celda_meta:
            self._publish_twist(Twist())
            self._publish_event(EV.META, f'meta alcanzada en {self._grid.cell}')
            self._terminado = True
            self._set_state('META')
            return

        self._begin_avanzar_paralelo()
        self._set_state('AVANZAR_PARALELO')

    def _handle_meta(self):
        self._publish_twist(Twist())

    def _handle_detenido(self):
        self._publish_twist(Twist())

    # ------------------------------------------------------------------
    # Utilidades de publicacion
    # ------------------------------------------------------------------
    def _publish_twist(self, cmd: Twist):
        self._cmd_pub.publish(cmd)

    def _publish_event(self, tipo: str, detalle: str):
        evt = RobotEvent()
        evt.header.stamp = self.get_clock().now().to_msg()
        evt.tipo = tipo
        evt.detalle = detalle
        self._event_pub.publish(evt)
        self.get_logger().info(f'[{tipo}] {detalle}')

    def _set_state(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'estado: {self._state} -> {new_state}')
            self._state = new_state
        self._state_pub.publish(String(data=self._state))


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
