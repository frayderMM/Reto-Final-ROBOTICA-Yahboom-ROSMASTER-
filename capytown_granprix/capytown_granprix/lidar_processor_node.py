#!/usr/bin/env python3
"""Nodo de lectura LiDAR.

Se suscribe a ``/scan`` (sensor_msgs/LaserScan, LiDAR MS200) y publica
``/lidar_zones`` (capytown_interfaces/LidarZones) con la distancia
minima detectada en cada zona angular de interes: frente, derecha
delantera (S1), derecha lateral, derecha trasera (S2) e izquierda.

Este nodo NO decide nada: solo traduce la nube de puntos 360 grados en
cinco numeros que los demas nodos (wall_follower, state_machine)
consumen. Mantenerlo asi facilita calibrar el LiDAR (offset de
montaje, inversion izquierda/derecha) sin tocar la logica de control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

from capytown_interfaces.msg import LidarZones
from capytown_granprix.lidar_utils import ZoneWindow, compute_all_zones


class LidarProcessorNode(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/lidar_zones')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_left_right', False)
        self.declare_parameter('max_range_use_m', 4.0)
        self.declare_parameter('front_window_deg', [-15.0, 15.0])
        self.declare_parameter('right_front_window_deg', [-75.0, -45.0])
        self.declare_parameter('right_window_deg', [-110.0, -70.0])
        self.declare_parameter('right_rear_window_deg', [-135.0, -105.0])
        self.declare_parameter('left_window_deg', [70.0, 110.0])

        self._scan_topic = self.get_parameter('scan_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._front_offset_rad = self.get_parameter('front_offset_deg').value * 3.141592653589793 / 180.0
        self._sign = -1 if self.get_parameter('invert_left_right').value else 1
        self._max_range_use_m = float(self.get_parameter('max_range_use_m').value)

        self._windows = {
            'front': ZoneWindow(*self.get_parameter('front_window_deg').value),
            'right_front': ZoneWindow(*self.get_parameter('right_front_window_deg').value),
            'right': ZoneWindow(*self.get_parameter('right_window_deg').value),
            'right_rear': ZoneWindow(*self.get_parameter('right_rear_window_deg').value),
            'left': ZoneWindow(*self.get_parameter('left_window_deg').value),
        }

        self._pub = self.create_publisher(
            LidarZones, self._output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self._sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            f'lidar_processor listo: {self._scan_topic} -> {self._output_topic} '
            f'(offset={self.get_parameter("front_offset_deg").value} deg, '
            f'invertido={self.get_parameter("invert_left_right").value})'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        range_max_use = min(msg.range_max, self._max_range_use_m)
        zones = compute_all_zones(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=range_max_use,
            front_offset_rad=self._front_offset_rad,
            sign=self._sign,
            windows=self._windows,
        )

        out = LidarZones()
        out.header = msg.header

        front_d, front_v = zones['front']
        rf_d, rf_v = zones['right_front']
        r_d, r_v = zones['right']
        rr_d, rr_v = zones['right_rear']
        left_d, left_v = zones['left']

        out.front = front_d
        out.front_valid = front_v
        out.right_front = rf_d
        out.right_front_valid = rf_v
        out.right = r_d
        out.right_valid = r_v
        out.right_rear = rr_d
        out.right_rear_valid = rr_v
        out.left = left_d
        out.left_valid = left_v

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
