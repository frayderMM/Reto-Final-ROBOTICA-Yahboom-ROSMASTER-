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
    -> DECIDIR -> PAUSA_GIRO -> GIRAR -> ALINEAR -> VERIFICAR_META
    -> (META o vuelve a AVANZAR_PARALELO)

    PAUSA_GIRO (fuera de la lista original del documento de referencia)
    es una espera fija de ``tiempo_pausa_antes_girar_s`` con el robot
    detenido entre "ya decidi" y "empiezo a girar", para que el giro se
    vea como un movimiento separado del avance.

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
        self._pausa_giro_start = None

        self._esperando_obstaculo = False
        self._espera_obstaculo_inicio = None
        self._contador_frente_dos_reglas = 0
        self._yaw_inicio_giro = 0.0

        self._STATE_HANDLERS = {
            'INICIAR': self._handle_iniciar,
            'AVANZAR_PARALELO': self._handle_avanzar_paralelo,
            'DETECTAR_CRUCE': self._handle_detectar_cruce,
            'BUSCAR_PARE': self._handle_buscar_pare,
            'DECIDIR': self._handle_decidir,
            'PAUSA_GIRO': self._handle_pausa_giro,
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
            # Modo de prueba: si es true, se saltan DETECTAR_CRUCE y
            # BUSCAR_PARE -- decide con una lectura unica (sin
            # confirmar con varias muestras). ALINEAR SI corre (no se
            # salta): es el paso que corrige el giro contra la pared
            # real via LiDAR en vez de confiar solo en el angulo
            # objetivo fijo + odometria. Util para calibrar el giro de
            # forma aislada, con el feedback de alineacion incluido.
            'modo_simplificado': False,
            # SOLO DOS REGLAS (rama logica-alternativa, para probar en
            # hardware lo mismo que sim_local/run_sim_laberinto.py::
            # _correr_logica_simple): avanzar recto mientras el frente
            # este libre; si hay obstaculo al frente, girar 90 grados a
            # la IZQUIERDA fijo (sin mirar derecha/izquierda, sin
            # seguir pared, sin celda, sin ALINEAR). Cuando esta en
            # true, IGNORA modo_simplificado y el resto de la maquina
            # de estados de cruce/PARE -- dejar en false para la
            # corrida real de competencia.
            'logica_dos_reglas': True,
            'velocidad_recta_mps': 0.15,
            # Correccion lateral de logica_dos_reglas: usa el AJUSTE DE
            # LINEA (right_line_*, angulo + distancia) en vez de la
            # distancia puntual right_valid/right -- evita confundir una
            # pared vista en diagonal con un obstaculo nuevo, porque el
            # ajuste de linea da el angulo real de la pared en vez de un
            # numero suelto. Misma formula que wall_follow_control.
            # calcular_comando, re-declarada aqui porque este modo NO
            # usa wall_follow_cmd en absoluto.
            'distancia_objetivo_m': 0.12,
            'ganancia_angulo_recta': 2.0,
            'ganancia_distancia_recta': 2.0,
            'angular_max_recta_radps': 0.6,
            # Confirmacion de N ciclos seguidos con front_narrow
            # bloqueado antes de girar -- un solo vistazo diagonal de un
            # ciclo (100% ruido/transitorio) no alcanza para disparar un
            # giro, tiene que sostenerse.
            'frente_confirmaciones_ciclos': 3,
            # Giro DINAMICO de logica_dos_reglas: no gira a angulo_giro_deg
            # fijo -- gira hasta quedar paralelo a la pared siguiente
            # (right_line_angle_rad ~0, con tolerancia_giro_deg), leyendo
            # la linea EN VIVO durante el giro. angulo_minimo_giro_deg es
            # resguardo (no detectar "paralelo" antes de girar al menos
            # esto, para no confundirse con la pared VIEJA); angulo_maximo_
            # giro_deg es tope de seguridad si nunca encuentra pared.
            'angulo_minimo_giro_deg': 45.0,
            'angulo_maximo_giro_deg': 150.0,
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
            # Angulo objetivo de giro para DERECHA/IZQUIERDA (ATRAS
            # siempre es 180, no usa este valor). 90 es el giro "real"
            # de una esquina en grilla; un poco mas (ej. 95) compensa
            # que el arco Ackermann suele quedar corto del objetivo.
            'angulo_giro_deg': 90.0,
            # Pausa fija (segundos) con el robot detenido entre DECIDIR
            # (ya sabe que va a girar) y el inicio del arco de GIRAR --
            # pedido para que el giro sea un movimiento claramente
            # separado del avance, no una transicion instantanea.
            'tiempo_pausa_antes_girar_s': 1.0,
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
        self._modo_simplificado = bool(g('modo_simplificado'))
        self._logica_dos_reglas = bool(g('logica_dos_reglas'))
        self._velocidad_recta = float(g('velocidad_recta_mps'))
        self._distancia_objetivo_recta = float(g('distancia_objetivo_m'))
        self._ganancia_angulo_recta = float(g('ganancia_angulo_recta'))
        self._ganancia_distancia_recta = float(g('ganancia_distancia_recta'))
        self._angular_max_recta = float(g('angular_max_recta_radps'))
        self._frente_confirmaciones_ciclos = int(g('frente_confirmaciones_ciclos'))
        self._contador_frente_dos_reglas = 0
        self._angulo_minimo_giro_rad = math.radians(float(g('angulo_minimo_giro_deg')))
        self._angulo_maximo_giro_rad = math.radians(float(g('angulo_maximo_giro_deg')))

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
        self._angulo_giro_rad = math.radians(float(g('angulo_giro_deg')))
        self._tiempo_pausa_antes_girar = float(g('tiempo_pausa_antes_girar_s'))

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
        if self._logica_dos_reglas:
            self._handle_avanzar_paralelo_dos_reglas()
            return

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
                return

            if self._modo_simplificado:
                # Decidir con una sola lectura, sin confirmar con varias
                # muestras ni pasar por BUSCAR_PARE.
                self._derecha_libre = bool(z.right_valid and z.right > self._umbral_lado_libre)
                self._frente_libre = bool(z.front_valid and z.front > self._umbral_frente_libre)
                self._izquierda_libre = bool(z.left_valid and z.left > self._umbral_lado_libre)
                self._set_state('DECIDIR')
            else:
                self._set_state('DETECTAR_CRUCE')
            return

        self._publish_twist(self._wall_follow_cmd)

    def _handle_avanzar_paralelo_dos_reglas(self):
        """TRES REGLAS (ver logica_dos_reglas arriba), con AJUSTE DE
        LINEA para el lado derecho (no distancia puntual) y
        confirmacion de varios ciclos para el frente:

        1. Avanzar recto mientras el frente este libre.
        2. Si hay ajuste de linea valido (right_line_*) de la pared
           derecha, corregir con Kp (angulo + distancia hacia
           distancia_objetivo_m) -- distingue una pared vista en
           diagonal (se corrige el angulo) de un obstaculo nuevo (no
           encaja como continuacion de esa recta). Si NO hay ajuste
           valido (se perdio la pared), avanzar recto sin corregir
           nada -- simple a proposito, sin heading-hold ni respaldo.
        3. Si hay obstaculo al frente (front_narrow, cono angosto)
           sostenido durante frente_confirmaciones_ciclos seguidos (no
           un solo vistazo), girar 90 grados a la IZQUIERDA (fijo,
           ignora derecha/izquierda) y retomar.

        No cuenta celdas ni pasa por ALINEAR -- portado tal cual de
        sim_local/run_sim_laberinto.py::_correr_logica_simple.
        """
        z = self._zones
        frente_cerca_1_ciclo = z.front_narrow_valid and z.front_narrow < self._umbral_frente_pared
        self._contador_frente_dos_reglas = (
            self._contador_frente_dos_reglas + 1 if frente_cerca_1_ciclo else 0
        )

        if self._contador_frente_dos_reglas >= self._frente_confirmaciones_ciclos:
            self._contador_frente_dos_reglas = 0
            self._decision_actual = 'IZQUIERDA'
            self._yaw_inicio_giro = self._yaw
            self._publish_event(
                EV.GIRO, f'obstaculo al frente ({z.front_narrow:.2f}m) -> IZQUIERDA'
            )
            self._publish_twist(Twist())
            self._pausa_giro_start = self.get_clock().now()
            self._set_state('PAUSA_GIRO')
            return

        cmd = Twist()
        if not z.right_line_valid:
            # Sin pared de referencia: avanzar recto, sin corregir nada.
            cmd.linear.x = self._velocidad_recta
            self._publish_twist(cmd)
            return
        error_distancia = self._distancia_objetivo_recta - z.right_line_distance_m
        correccion = (self._ganancia_angulo_recta * z.right_line_angle_rad
                      + self._ganancia_distancia_recta * error_distancia)
        cmd.linear.x = self._velocidad_recta
        cmd.angular.z = max(-self._angular_max_recta, min(self._angular_max_recta, correccion))
        self._publish_twist(cmd)

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
            if self._modo_simplificado:
                self._begin_avanzar_paralelo()
                self._set_state('AVANZAR_PARALELO')
            else:
                self._alinear_start = None
                self._set_state('ALINEAR')
            return

        self._giro_objetivo = self._compute_turn_target(self._yaw, direction)
        self._publish_event(EV.GIRO, f'{direction} desde {self._grid.cell}')
        self._publish_twist(Twist())
        self._pausa_giro_start = self.get_clock().now()
        self._set_state('PAUSA_GIRO')

    def _handle_pausa_giro(self):
        """Robot detenido ``tiempo_pausa_antes_girar_s`` antes de arrancar
        el arco de GIRAR -- separa visiblemente "termine de avanzar" de
        "empiezo a girar" en vez de una transicion instantanea."""
        self._publish_twist(Twist())
        elapsed = (self.get_clock().now() - self._pausa_giro_start).nanoseconds / 1e9
        if elapsed >= self._tiempo_pausa_antes_girar:
            self._set_state('GIRAR')

    def _compute_turn_target(self, yaw: float, direction: str) -> float:
        if direction == 'DERECHA':
            delta = -self._angulo_giro_rad
        elif direction == 'IZQUIERDA':
            delta = self._angulo_giro_rad
        elif direction == 'ATRAS':
            delta = math.pi
        else:
            delta = 0.0
        return normalize_angle(yaw + delta)

    def _handle_girar(self):
        if self._logica_dos_reglas:
            self._handle_girar_dinamico()
            return

        error = angle_diff(self._giro_objetivo, self._yaw)

        if abs(error) <= self._tolerancia_giro_rad:
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)
            # ALINEAR corre siempre, incluso en modo_simplificado: GIRAR
            # por si solo solo cierra el lazo contra el yaw de odometria
            # (un angulo objetivo fijo, con la deriva propia del
            # odometro pese al factor de correccion). ALINEAR corrige
            # ese resultado con el LiDAR real (right_front/right_rear)
            # despues del giro -- es el feedback real, no un angulo fijo.
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

    def _handle_girar_dinamico(self):
        """Giro DINAMICO (logica_dos_reglas): no gira a un angulo fijo
        -- sigue girando, leyendo la linea de la pared derecha EN VIVO
        (no ciego como el giro fijo), hasta quedar paralelo a la pared
        siguiente (right_line_angle_rad ~0), en vez de confiar en un
        angulo objetivo de odometria. Reemplaza GIRAR+ALINEAR por un
        solo movimiento continuo.

        angulo_minimo_giro_deg: resguardo -- recien despues de girar
        al menos esto (por odometria) se puede detectar "paralelo";
        si no, al arrancar el giro puede seguir viendo la pared VIEJA
        (la que seguia antes del obstaculo) casi paralela y pararia
        de inmediato sin girar nada.
        angulo_maximo_giro_deg: tope de seguridad si nunca encuentra
        una pared paralela (p.ej. queda mirando a un espacio abierto).
        """
        z = self._zones
        angulo_girado = abs(angle_diff(self._yaw, self._yaw_inicio_giro))

        if angulo_girado >= self._angulo_minimo_giro_rad:
            if z.right_line_valid and abs(z.right_line_angle_rad) <= self._tolerancia_giro_rad:
                self._publish_twist(Twist())
                self._grid.apply_turn(self._decision_actual)
                self.get_logger().info(
                    f'GIRO TERMINADO (paralelo): girado={math.degrees(angulo_girado):.0f} deg'
                )
                self._set_state('AVANZAR_PARALELO')
                return

        if angulo_girado >= self._angulo_maximo_giro_rad:
            self._publish_twist(Twist())
            self._grid.apply_turn(self._decision_actual)
            self.get_logger().info(
                f'GIRO TERMINADO (tope de seguridad, sin pared paralela): '
                f'girado={math.degrees(angulo_girado):.0f} deg'
            )
            self._set_state('AVANZAR_PARALELO')
            return

        cmd = Twist()
        cmd.linear.x = self._v_giro_lineal
        cmd.angular.z = self._v_giro_angular if self._decision_actual == 'IZQUIERDA' else -self._v_giro_angular
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
