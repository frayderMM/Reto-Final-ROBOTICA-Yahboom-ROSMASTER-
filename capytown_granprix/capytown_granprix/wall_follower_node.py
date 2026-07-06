#!/usr/bin/env python3
"""Nodo de control de seguimiento de pared derecha (AVANZAR_PARALELO).

Se suscribe a ``/lidar_zones`` y publica una velocidad SUGERIDA en
``/wall_follow/cmd_vel_suggestion`` (geometry_msgs/Twist). Este nodo
NO escribe directamente en ``/cmd_vel``: el nodo de decision
(state_machine_node) es el unico que actua sobre el robot, y solo
reenvia esta sugerencia mientras el estado sea AVANZAR_PARALELO. Esto
evita que dos nodos publiquen comandos de movimiento en simultaneo.

Logica de control (ver logica_pared_derecha_robot.md, secciones 6-8 y
19): se usan dos zonas del lado derecho -- S1 (right_front) y S2
(right_rear) -- para mantener al robot PARALELO a la pared antes de
corregir la distancia. Corregir el angulo primero evita que el robot
entre en diagonal.

Convencion de signos de angular.z (REP-103): positivo = giro hacia la
izquierda (antihorario), negativo = giro hacia la derecha (horario).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist

from capytown_interfaces.msg import LidarZones
from capytown_granprix.geometry_utils import clamp


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.declare_parameter('lidar_zones_topic', '/lidar_zones')
        self.declare_parameter('output_topic', '/wall_follow/cmd_vel_suggestion')
        self.declare_parameter('distancia_min_m', 0.05)
        self.declare_parameter('distancia_max_m', 0.12)
        self.declare_parameter('tolerancia_angulo_m', 0.03)
        self.declare_parameter('velocidad_lineal_mps', 0.15)
        self.declare_parameter('ganancia_angulo', 3.0)
        self.declare_parameter('ganancia_distancia', 2.0)
        self.declare_parameter('angular_max_radps', 0.6)
        self.declare_parameter('frente_minimo_seguro_m', 0.15)

        self._zones_topic = self.get_parameter('lidar_zones_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._distancia_min = float(self.get_parameter('distancia_min_m').value)
        self._distancia_max = float(self.get_parameter('distancia_max_m').value)
        self._tolerancia_angulo = float(self.get_parameter('tolerancia_angulo_m').value)
        self._v_base = float(self.get_parameter('velocidad_lineal_mps').value)
        self._k_angulo = float(self.get_parameter('ganancia_angulo').value)
        self._k_distancia = float(self.get_parameter('ganancia_distancia').value)
        self._angular_max = float(self.get_parameter('angular_max_radps').value)
        self._frente_minimo = float(self.get_parameter('frente_minimo_seguro_m').value)

        self._pub = self.create_publisher(Twist, self._output_topic, 10)
        self._sub = self.create_subscription(
            LidarZones, self._zones_topic, self._on_zones, QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            f'wall_follower listo: rango={self._distancia_min:.2f}-{self._distancia_max:.2f} m, '
            f'v_base={self._v_base:.2f} m/s'
        )

    def _on_zones(self, msg: LidarZones) -> None:
        cmd = Twist()

        if msg.front_valid and msg.front < self._frente_minimo:
            # Seguridad redundante: si hay pared muy cerca al frente,
            # no avanzar aunque el estado siga siendo AVANZAR_PARALELO
            # (el nodo de decision debera reaccionar en su propio ciclo).
            self._pub.publish(cmd)
            return

        if not (msg.right_front_valid and msg.right_rear_valid):
            # Sin referencia confiable de pared derecha (pasillo abierto):
            # avanzar recto sin corregir, evitando girar "a ciegas".
            cmd.linear.x = self._v_base
            cmd.angular.z = 0.0
            self._pub.publish(cmd)
            return

        error_angulo = msg.right_front - msg.right_rear

        if abs(error_angulo) > self._tolerancia_angulo:
            # 1. Prioridad: corregir paralelismo antes que distancia.
            correccion = -self._k_angulo * error_angulo
        else:
            # 2. Ya esta paralelo: corregir distancia solo si esta fuera
            # del rango aceptable [distancia_min, distancia_max]. Dentro
            # del rango, avanzar recto sin corregir (evita oscilar).
            distancia_promedio = (msg.right_front + msg.right_rear) / 2.0
            if distancia_promedio > self._distancia_max:
                # Muy lejos: acercarse hasta entrar al rango.
                error_distancia = self._distancia_max - distancia_promedio
            elif distancia_promedio < self._distancia_min:
                # Muy pegado: alejarse hasta entrar al rango.
                error_distancia = self._distancia_min - distancia_promedio
            else:
                error_distancia = 0.0
            correccion = self._k_distancia * error_distancia

        cmd.linear.x = self._v_base
        cmd.angular.z = clamp(correccion, -self._angular_max, self._angular_max)
        self._pub.publish(cmd)


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
