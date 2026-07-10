#!/usr/bin/env python3
"""Visualizador en vivo del LiDAR MS200 (diagnostico, no forma parte del
paquete ROS2 del reto). Dibuja con matplotlib los puntos crudos de
``/scan`` en el marco del ROBOT (frente arriba, izquierda a la
izquierda de la pantalla) y superpone 4 sectores angulares reducidos
(frente, derecha, izquierda, atras) con la distancia minima de cada
uno, para verificar visualmente si `front_offset_deg` / `invert_left_right`
estan bien calibrados antes de tocar el YAML del paquete principal.

El sector FRENTE usa el mismo cono angosto que logica_dos_reglas en
produccion (front_narrow_window_deg, ver granprix_params.yaml), no un
cono generico de referencia -- asi lo que se ve aca es EXACTAMENTE lo
que el robot real usa para decidir "obstaculo al frente". Cuando la
distancia frontal cae por debajo de --umbral-frente-pared-m, se
imprime un aviso en la terminal (una sola vez por evento, no en cada
frame) y el titulo del grafico se pone en rojo.

Uso (dentro del contenedor, con el workspace *no* necesariamente
compilado -- este script no depende de capytown_granprix):

    python3 /root/yahboomcar_ws/src/reto-final/lidar_viz.py
    python3 lidar_viz.py --ros-args -p front_offset_deg:=180.0 -p invert_left_right:=true

Requiere entorno grafico (VNC, `-e DISPLAY=:0` al entrar al contenedor).
"""

import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan


class LidarVizNode(Node):

    def __init__(self):
        super().__init__('lidar_viz')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_offset_deg', 0.0)
        self.declare_parameter('invert_left_right', False)
        self.declare_parameter('max_range_m', 2.5)
        # Mismos valores que front_narrow_window_deg / umbral_frente_
        # pared_m de granprix_params.yaml -- cambiar ahi tambien si se
        # ajustan en produccion, para que el viz siga mostrando el
        # cono REAL.
        self.declare_parameter('front_narrow_deg', 5.0)
        self.declare_parameter('umbral_frente_pared_m', 0.40)

        self.front_offset_rad = math.radians(self.get_parameter('front_offset_deg').value)
        self.sign = -1 if self.get_parameter('invert_left_right').value else 1
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.front_narrow_deg = float(self.get_parameter('front_narrow_deg').value)
        self.umbral_frente_pared = float(self.get_parameter('umbral_frente_pared_m').value)

        self.sectores = [
            ('FRENTE', -self.front_narrow_deg, self.front_narrow_deg, 'tab:green'),
            ('DERECHA', -110.0, -70.0, 'tab:red'),
            ('IZQUIERDA', 70.0, 110.0, 'tab:blue'),
            ('ATRAS', 160.0, -160.0, 'tab:orange'),  # cruza +-180
        ]
        self._frente_detectado = False

        self.last_scan = None
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg


def robot_frame_angles(scan: LaserScan, front_offset_rad: float, sign: int) -> np.ndarray:
    n = len(scan.ranges)
    idx = np.arange(n, dtype=float)
    a = scan.angle_min + idx * scan.angle_increment
    a = np.mod(a + math.pi, 2 * math.pi) - math.pi
    r = sign * (a - front_offset_rad)
    return np.mod(r + math.pi, 2 * math.pi) - math.pi


def zone_min_distance(ranges: np.ndarray, robot_angles: np.ndarray, range_min: float,
                       range_max: float, lo_deg: float, hi_deg: float) -> float:
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    if lo <= hi:
        mask = (robot_angles >= lo) & (robot_angles <= hi)
    else:
        mask = (robot_angles >= lo) | (robot_angles <= hi)
    mask &= np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    return float(np.min(ranges[mask])) if np.any(mask) else float('inf')


def to_plot_xy(ranges: np.ndarray, robot_angles: np.ndarray):
    """Frente arriba, izquierda a la izquierda de la pantalla."""
    x = ranges * np.sin(-robot_angles)
    y = ranges * np.cos(robot_angles)
    return x, y


def main():
    rclpy.init()
    node = LidarVizNode()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    print('Ctrl+C en esta terminal para cerrar.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            scan = node.last_scan
            if scan is not None:
                ranges = np.asarray(scan.ranges, dtype=float)
                robot_angles = robot_frame_angles(scan, node.front_offset_rad, node.sign)
                x, y = to_plot_xy(ranges, robot_angles)

                ax.clear()
                ax.scatter(x, y, s=4, c='black')
                ax.plot(0, 0, marker='s', markersize=12, color='dimgray')  # robot
                ax.annotate('FRENTE', (0, node.max_range * 0.95), ha='center')

                frente_dist = None
                for nombre, lo_deg, hi_deg, color in node.sectores:
                    d = zone_min_distance(ranges, robot_angles, scan.range_min,
                                           min(scan.range_max, node.max_range), lo_deg, hi_deg)
                    if nombre == 'FRENTE':
                        frente_dist = d
                    for ang_deg in (lo_deg, hi_deg):
                        ang = math.radians(ang_deg)
                        bx = node.max_range * math.sin(-ang)
                        by = node.max_range * math.cos(ang)
                        ax.plot([0, bx], [0, by], color=color, linewidth=1, alpha=0.6)
                    mid = math.radians((lo_deg + hi_deg) / 2.0 if lo_deg <= hi_deg
                                        else (lo_deg + hi_deg + 360.0) / 2.0)
                    tx = node.max_range * 0.55 * math.sin(-mid)
                    ty = node.max_range * 0.55 * math.cos(mid)
                    etiqueta = f'{nombre}\n{d:.2f} m' if math.isfinite(d) else f'{nombre}\n---'
                    ax.text(tx, ty, etiqueta, color=color, ha='center', fontsize=9, weight='bold')

                # Obstaculo al frente: mismo criterio que state_machine_
                # node (front_narrow + umbral_frente_pared_m). Se avisa
                # UNA VEZ por evento (al entrar y al salir), no en cada
                # frame -- si no, serian decenas de prints por segundo.
                frente_bloqueado = frente_dist is not None and frente_dist < node.umbral_frente_pared
                if frente_bloqueado and not node._frente_detectado:
                    print(f'>>> OBSTACULO AL FRENTE detectado a {frente_dist:.2f} m '
                          f'(cono +-{node.front_narrow_deg:.0f} deg, umbral {node.umbral_frente_pared:.2f} m)')
                elif not frente_bloqueado and node._frente_detectado:
                    print('    frente despejado de nuevo')
                node._frente_detectado = frente_bloqueado

                titulo = 'LiDAR MS200 - marco del robot (frente = arriba)'
                if frente_bloqueado:
                    titulo += f'  ***OBSTACULO AL FRENTE ({frente_dist:.2f} m)***'
                ax.set_title(titulo, color='tab:red' if frente_bloqueado else 'black')

                ax.set_xlim(-node.max_range, node.max_range)
                ax.set_ylim(-node.max_range, node.max_range)
                ax.set_aspect('equal')
                ax.grid(True, linestyle=':', alpha=0.4)

            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
