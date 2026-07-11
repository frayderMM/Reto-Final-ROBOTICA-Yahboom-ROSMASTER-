#!/usr/bin/env python3
"""Emisor de datos para el tablero web de diagnostico (corre en el robot).

Arquitectura (adaptada de un dashboard similar, para no cargar la Pi
ni depender de VNC/matplotlib):
  · ROBOT (Pi): este nodo se suscribe a los topicos ROS y EMITE los
    datos por HTTP como JSON liviano en ``GET /data`` (con CORS
    habilitado). No dibuja nada -- solo serializa.
  · LAPTOP: abre ``web/dashboard.html`` (frontend puro con Canvas) que
    hace fetch a ``http://<IP-del-robot>:8080/data`` y DIBUJA todo en
    el navegador. El render pesado ocurre en la laptop, no en la Pi.

A diferencia de otros visualizadores de este tipo, este NO intenta
reconstruir la posicion absoluta en la grilla del laberinto (eso
requiere constantes de calibracion muy especificas de un layout
fisico dado, que no tenemos validadas para esta pista) -- en cambio
muestra directamente lo que los nodos ya calculan: la trayectoria
acumulada en el marco de ODOMETRIA (relativa al arranque), el LiDAR
crudo en el marco del ROBOT, y las zonas ya procesadas por
lidar_processor_node (front/right/left, right_line_*/left_line_*).

Uso (en el robot):
    ros2 run capytown_granprix web_dashboard_node
En la laptop: abrir web/dashboard.html e ingresar la IP del robot.
"""

import base64
import json
import math
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from capytown_interfaces.msg import LidarZones, RobotEvent
from capytown_granprix.geometry_utils import yaw_from_quaternion

try:
    import cv2
    import numpy as np
    _OPENCV_DISPONIBLE = True
except Exception:  # noqa: BLE001 - opencv/numpy pueden faltar en algunos entornos
    cv2 = None
    np = None
    _OPENCV_DISPONIBLE = False

TRAYECTORIA_PASO_MIN_M = 0.02   # decimado: no agregar puntos mas seguido que esto
TRAYECTORIA_MAX_PUNTOS = 4000
EVENTOS_MAX = 60


def _imagen_a_jpeg_data_url(msg: Image, max_width: int, calidad: int) -> str | None:
    """Convierte un sensor_msgs/Image a data URL JPEG para el navegador."""
    encoding = (msg.encoding or '').lower()
    if encoding in ('mjpeg', 'jpeg'):
        return 'data:image/jpeg;base64,' + base64.b64encode(bytes(msg.data)).decode('ascii')
    if not _OPENCV_DISPONIBLE:
        return None
    try:
        h, w = int(msg.height), int(msg.width)
        if h <= 0 or w <= 0:
            return None
        if encoding in ('rgb8', 'bgr8'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            if encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            return None
        if w > max_width > 0:
            escala = max_width / float(w)
            img = cv2.resize(img, (max_width, max(1, int(h * escala))),
                              interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), calidad])
        if not ok:
            return None
        return 'data:image/jpeg;base64,' + base64.b64encode(buf.tobytes()).decode('ascii')
    except Exception:  # noqa: BLE001 - cualquier frame corrupto se descarta, no se cae el nodo
        return None


class WebDashboardNode(Node):

    def __init__(self):
        super().__init__('web_dashboard')

        self.declare_parameter('puerto', 8080)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('lidar_zones_topic', '/lidar_zones')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_state_topic', '/robot_state')
        self.declare_parameter('event_topic', '/robot_event')
        self.declare_parameter('pare_topic', '/pare_detectado')
        self.declare_parameter('meta_topic', '/meta_detectado')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('usar_camara', True)
        self.declare_parameter('camera_max_width', 360)
        # FPS bajo para no saturar la Pi/la red con JPEG+base64; la
        # camara fisica sigue publicando a su tasa normal para el
        # detector de colores, esto solo limita lo que se emite aca.
        self.declare_parameter('camera_fps', 10.0)
        self.declare_parameter('camera_jpeg_quality', 45)
        self.declare_parameter('lidar_range_max_m', 3.0)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self._puerto = int(gp('puerto'))
        self._usar_camara = bool(gp('usar_camara'))
        self._camera_topic = str(gp('camera_topic'))
        self._camera_max_width = int(gp('camera_max_width'))
        camera_fps = float(gp('camera_fps'))
        self._camera_periodo_s = 1.0 / camera_fps if camera_fps > 0.0 else 0.0
        self._camera_calidad = int(gp('camera_jpeg_quality'))
        self._lidar_range_max = float(gp('lidar_range_max_m'))

        # ── Estado compartido (protegido por lock, lo lee el hilo HTTP) ──
        self._lock = threading.Lock()
        self._estado = 'INICIANDO'
        self._eventos = deque(maxlen=EVENTOS_MAX)
        self._pare_detectado = False
        self._meta_detectado = False
        self._lidar_pts = []            # [[x,y], ...] marco del robot
        self._zonas = {}                # ultimo LidarZones, como dict plano
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._odom_origen = None        # (x0, y0, yaw0) del primer /odom_raw
        self._odom_ultimo = None        # (x, y) crudo, para decimar la trayectoria
        self._trayectoria = deque(maxlen=TRAYECTORIA_MAX_PUNTOS)
        self._pos_rel = [0.0, 0.0]
        self._yaw_rel = 0.0
        self._camera_src = None
        self._camera_info = {'ok': False, 'topic': self._camera_topic}
        self._camera_ultimo_emit_s = 0.0

        # ── Suscripciones ────────────────────────────────────────────────
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(LaserScan, str(gp('scan_topic')), self._on_scan, qos)
        self.create_subscription(LidarZones, str(gp('lidar_zones_topic')), self._on_zones, qos)
        self.create_subscription(Odometry, str(gp('odom_topic')), self._on_odom, 10)
        self.create_subscription(Twist, str(gp('cmd_vel_topic')), self._on_cmd_vel, 10)
        self.create_subscription(String, str(gp('robot_state_topic')), self._on_estado, 10)
        self.create_subscription(RobotEvent, str(gp('event_topic')), self._on_evento, 10)
        self.create_subscription(Bool, str(gp('pare_topic')), self._on_pare, 10)
        self.create_subscription(Bool, str(gp('meta_topic')), self._on_meta, 10)
        if self._usar_camara:
            self.create_subscription(Image, self._camera_topic, self._on_camera, qos)

        self._arrancar_http()
        self.get_logger().info(
            f'web_dashboard listo en el puerto {self._puerto}. En la laptop abri '
            f'web/dashboard.html y usa la IP de este robot (GET /data).'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        pts = []
        n = len(msg.ranges)
        paso = max(1, n // 360)  # decimado: como mucho ~360 puntos
        for i in range(0, n, paso):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > self._lidar_range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            pts.append([round(r * math.cos(ang), 3), round(r * math.sin(ang), 3)])
        with self._lock:
            self._lidar_pts = pts

    def _on_zones(self, msg: LidarZones):
        with self._lock:
            self._zonas = {
                'front': msg.front, 'front_valid': bool(msg.front_valid),
                'right': msg.right, 'right_valid': bool(msg.right_valid),
                'left': msg.left, 'left_valid': bool(msg.left_valid),
                'front_narrow': msg.front_narrow, 'front_narrow_valid': bool(msg.front_narrow_valid),
                'right_line_angle_deg': math.degrees(msg.right_line_angle_rad),
                'right_line_distance_m': msg.right_line_distance_m,
                'right_line_valid': bool(msg.right_line_valid),
                'left_line_angle_deg': math.degrees(msg.left_line_angle_rad),
                'left_line_distance_m': msg.left_line_distance_m,
                'left_line_valid': bool(msg.left_line_valid),
            }

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            if self._odom_origen is None:
                self._odom_origen = (p.x, p.y, yaw)
                self._odom_ultimo = (p.x, p.y)
                self._trayectoria.append([0.0, 0.0])
                return

            x0, y0, yaw0 = self._odom_origen
            # Posicion relativa al arranque, rotada al marco INICIAL del
            # robot (no al marco mundo de /odom_raw) para que "adelante"
            # en el dibujo sea "adelante" al arrancar.
            dx, dy = p.x - x0, p.y - y0
            rel_x = dx * math.cos(-yaw0) - dy * math.sin(-yaw0)
            rel_y = dx * math.sin(-yaw0) + dy * math.cos(-yaw0)
            self._pos_rel = [round(rel_x, 3), round(rel_y, 3)]
            self._yaw_rel = round(yaw - yaw0, 4)

            ultimo = self._odom_ultimo
            if ultimo is None or math.hypot(p.x - ultimo[0], p.y - ultimo[1]) >= TRAYECTORIA_PASO_MIN_M:
                self._trayectoria.append(list(self._pos_rel))
                self._odom_ultimo = (p.x, p.y)

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._cmd_v = round(msg.linear.x, 3)
            self._cmd_w = round(msg.angular.z, 3)

    def _on_estado(self, msg: String):
        with self._lock:
            self._estado = msg.data

    def _on_evento(self, msg: RobotEvent):
        with self._lock:
            self._eventos.append({'tipo': msg.tipo, 'detalle': msg.detalle})

    def _on_pare(self, msg: Bool):
        with self._lock:
            self._pare_detectado = bool(msg.data)

    def _on_meta(self, msg: Bool):
        with self._lock:
            self._meta_detectado = bool(msg.data)

    def _on_camera(self, msg: Image):
        ahora_s = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            if (self._camera_periodo_s > 0.0
                    and ahora_s - self._camera_ultimo_emit_s < self._camera_periodo_s):
                return
            self._camera_ultimo_emit_s = ahora_s
        src = _imagen_a_jpeg_data_url(msg, self._camera_max_width, self._camera_calidad)
        with self._lock:
            self._camera_info = {
                'ok': src is not None, 'topic': self._camera_topic,
                'encoding': msg.encoding, 'w': int(msg.width), 'h': int(msg.height),
            }
            if src is not None:
                self._camera_src = src

    # ------------------------------------------------------------------
    # Snapshot JSON para el frontend
    # ------------------------------------------------------------------
    def _snapshot(self) -> dict:
        with self._lock:
            return {
                'estado': self._estado,
                'eventos': list(self._eventos),
                'pare_detectado': self._pare_detectado,
                'meta_detectado': self._meta_detectado,
                'lidar': list(self._lidar_pts),
                'zonas': dict(self._zonas),
                'cmd_v': self._cmd_v,
                'cmd_w': self._cmd_w,
                'pos_rel': list(self._pos_rel),
                'yaw_rel_deg': round(math.degrees(self._yaw_rel), 1),
                'trayectoria': list(self._trayectoria),
                'camera': dict(self._camera_info, src=self._camera_src),
            }

    def _arrancar_http(self):
        nodo = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):
                pass  # silenciar el log de cada request

            def _cors(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-store')

            def do_GET(self):
                if self.path.startswith('/data'):
                    cuerpo = json.dumps(nodo._snapshot()).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self._cors()
                    self.send_header('Content-Length', str(len(cuerpo)))
                    self.end_headers()
                    self.wfile.write(cuerpo)
                else:
                    cuerpo = (b'web_dashboard activo. El frontend esta en la laptop '
                              b'(web/dashboard.html). Datos en /data')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/plain; charset=utf-8')
                    self._cors()
                    self.send_header('Content-Length', str(len(cuerpo)))
                    self.end_headers()
                    self.wfile.write(cuerpo)

        self._srv = ThreadingHTTPServer(('0.0.0.0', self._puerto), Handler)
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
