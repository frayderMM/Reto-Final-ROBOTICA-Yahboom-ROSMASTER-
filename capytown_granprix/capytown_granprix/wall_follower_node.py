#!/usr/bin/env python3
"""Nodo de control de seguimiento de pared derecha (AVANZAR_PARALELO).

Se suscribe a ``/lidar_zones`` y publica una velocidad SUGERIDA en
``/wall_follow/cmd_vel_suggestion`` (geometry_msgs/Twist). Este nodo
NO escribe directamente en ``/cmd_vel``: el nodo de decision
(state_machine_node) es el unico que actua sobre el robot, y solo
reenvia esta sugerencia mientras el estado sea AVANZAR_PARALELO. Esto
evita que dos nodos publiquen comandos de movimiento en simultaneo.

Logica de control -- REGRESION DE LINEA + Kp (``right_line_*`` de
``lidar_processor_node``, ajuste por minimos cuadrados a todos los
puntos del lado derecho, no solo 2 como el metodo original de
S1/S2). Se corrige primero el ANGULO de la recta (paralelismo) y
despues la DISTANCIA perpendicular hacia ``distancia_objetivo_m``,
igual prioridad que el metodo de 2 puntos pero mas robusto al ruido.
Validado antes en ``sim_local/`` (simulador local sin ROS2) --
incluye la derivacion geometrica del signo de la correccion de
angulo, ver comentario en ``_on_zones``.

Convencion de signos de angular.z (REP-103): positivo = giro hacia la
izquierda (antihorario), negativo = giro hacia la derecha (horario).

Cuando no hay pared derecha de referencia (pasillo abierto), se usa un
control Kp de heading (con el yaw de ``/odom_raw``) para mantener el
rumbo recto en vez de simplemente anular la correccion -- evita que un
sesgo mecanico del chasis lo desvie lentamente sin que nada lo corrija.

Modo de prueba (``publicar_directo_en_cmd_vel``): para calibrar SOLO el
seguimiento recto, sin que ``state_machine_node`` interrumpa con fases
de celda/cruce/giro, este nodo puede publicar la misma velocidad
directo en ``/cmd_vel`` ademas de la sugerencia normal. Util para
correr unicamente ``lidar_processor_node`` + ``wall_follower_node`` en
un pasillo largo. NO usar este modo junto con ``state_machine_node``
corriendo (dos nodos escribirian en ``/cmd_vel`` a la vez).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from capytown_interfaces.msg import LidarZones
from capytown_granprix.geometry_utils import angle_diff, clamp, yaw_from_quaternion


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.declare_parameter('lidar_zones_topic', '/lidar_zones')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('output_topic', '/wall_follow/cmd_vel_suggestion')
        self.declare_parameter('distancia_objetivo_m', 0.12)
        self.declare_parameter('tolerancia_angulo_deg', 3.0)
        self.declare_parameter('velocidad_lineal_mps', 0.15)
        self.declare_parameter('ganancia_angulo', 2.0)
        self.declare_parameter('ganancia_distancia', 2.0)
        self.declare_parameter('ganancia_heading', 2.0)
        self.declare_parameter('angular_max_radps', 0.6)
        self.declare_parameter('frente_minimo_seguro_m', 0.15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publicar_directo_en_cmd_vel', False)

        self._zones_topic = self.get_parameter('lidar_zones_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._distancia_objetivo = float(self.get_parameter('distancia_objetivo_m').value)
        self._tolerancia_angulo_rad = math.radians(
            float(self.get_parameter('tolerancia_angulo_deg').value)
        )
        self._v_base = float(self.get_parameter('velocidad_lineal_mps').value)
        self._k_angulo = float(self.get_parameter('ganancia_angulo').value)
        self._k_distancia = float(self.get_parameter('ganancia_distancia').value)
        self._k_heading = float(self.get_parameter('ganancia_heading').value)
        self._angular_max = float(self.get_parameter('angular_max_radps').value)
        self._frente_minimo = float(self.get_parameter('frente_minimo_seguro_m').value)
        self._publicar_directo = bool(self.get_parameter('publicar_directo_en_cmd_vel').value)

        self._yaw = 0.0
        self._heading_objetivo = None

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._cmd_vel_pub = None
        if self._publicar_directo:
            cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
            self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.get_logger().warn(
                f'MODO DE PRUEBA activo: publicando directo en {cmd_vel_topic}. '
                'No correr junto con state_machine_node.'
            )

        self._sub = self.create_subscription(
            LidarZones, self._zones_topic, self._on_zones, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self.get_logger().info(
            f'wall_follower listo: objetivo={self._distancia_objetivo * 100:.1f} cm, '
            f'v_base={self._v_base:.2f} m/s'
        )

    def _publish(self, cmd: Twist) -> None:
        self._pub.publish(cmd)
        if self._cmd_vel_pub is not None:
            self._cmd_vel_pub.publish(cmd)

    def _on_odom(self, msg: Odometry) -> None:
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_zones(self, msg: LidarZones) -> None:
        cmd = Twist()

        if msg.front_valid and msg.front < self._frente_minimo:
            # Seguridad redundante: si hay pared muy cerca al frente,
            # no avanzar aunque el estado siga siendo AVANZAR_PARALELO
            # (el nodo de decision debera reaccionar en su propio ciclo).
            self._publish(cmd)
            return

        if not msg.right_line_valid:
            # Sin referencia confiable de pared derecha (pasillo abierto):
            # mantener el rumbo con un Kp de heading sobre el yaw de
            # odometria, en vez de simplemente anular la correccion (eso
            # dejaba que un sesgo mecanico del chasis desviara el robot
            # sin que nada lo corrigiera).
            if self._heading_objetivo is None:
                self._heading_objetivo = self._yaw
            error_heading = angle_diff(self._heading_objetivo, self._yaw)
            correccion = self._k_heading * error_heading
            cmd.linear.x = self._v_base
            cmd.angular.z = clamp(correccion, -self._angular_max, self._angular_max)
            self._publish(cmd)
            return

        # Hay pared derecha valida: al recuperarla, olvidar el heading
        # objetivo anterior para que la proxima vez que se pierda la
        # pared se capture un rumbo fresco (no uno desactualizado).
        self._heading_objetivo = None

        if abs(msg.right_line_angle_rad) > self._tolerancia_angulo_rad:
            # 1. Prioridad: corregir paralelismo antes que distancia.
            #
            # Geometria (derivada y verificada en sim_local/ antes de
            # portarla aqui): si la pared es una recta horizontal en el
            # mundo y el robot tiene yaw theta respecto a ella, el
            # angulo que se ve EN EL MARCO DEL ROBOT es
            # right_line_angle_rad = atan(pendiente_local) = -theta.
            # Para corregir theta -> 0 se necesita angular.z = -k*theta
            # = +k*right_line_angle_rad (SIN signo negativo). Con el
            # signo cambiado, el lazo es de realimentacion POSITIVA y
            # el robot diverge en menos de 1 s.
            correccion = self._k_angulo * msg.right_line_angle_rad
        else:
            # 2. Ya esta paralelo: corregir distancia hacia el objetivo
            # (correccion continua, sin banda muerta -- una banda deja
            # al robot "flotar" sin corregir mientras este adentro, lo
            # que en la practica se ve como que no mantiene una
            # distancia consistente).
            error_distancia = self._distancia_objetivo - msg.right_line_distance_m
            correccion = self._k_distancia * error_distancia

        cmd.linear.x = self._v_base
        cmd.angular.z = clamp(correccion, -self._angular_max, self._angular_max)
        self._publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
