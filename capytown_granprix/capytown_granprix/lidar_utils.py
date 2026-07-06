"""Extraccion de zonas angulares de distancia a partir de un LaserScan.

El LiDAR MS200 es de 360 grados. Este modulo convierte el arreglo
``ranges`` de un ``sensor_msgs/LaserScan`` en distancias minimas por
zona (frente, derecha delantera S1, derecha lateral, derecha trasera
S2, izquierda), en el marco de referencia del ROBOT (no del sensor).

La orientacion real del LiDAR respecto al frente del robot depende del
montaje fisico y se calibra con dos parametros (ver README, seccion de
calibracion):

- ``front_offset_rad``: angulo del scan que corresponde al frente real
  del robot (0 si el LiDAR ya reporta 0 rad = frente).
- ``sign``: +1 o -1. Corrige el sentido de giro (horario/antihorario)
  si izquierda y derecha aparecen invertidas.

Convencion de angulos del ROBOT (una vez aplicada la calibracion):
0 rad = frente, +90 grados = izquierda, -90 grados = derecha
(convencion estandar REP-103, antihoraria positiva).
"""

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class ZoneWindow:
    """Ventana angular [lo_deg, hi_deg] en el marco del robot.

    Si lo_deg > hi_deg se interpreta como una ventana que cruza el
    limite +-180 grados (wraparound).
    """

    lo_deg: float
    hi_deg: float


def _scan_angles(n_points: int, angle_min: float, angle_increment: float) -> np.ndarray:
    idx = np.arange(n_points, dtype=float)
    a = angle_min + idx * angle_increment
    return np.mod(a + math.pi, 2.0 * math.pi) - math.pi


def _robot_frame_angles(scan_angles: np.ndarray, front_offset_rad: float, sign: int) -> np.ndarray:
    r = sign * (scan_angles - front_offset_rad)
    return np.mod(r + math.pi, 2.0 * math.pi) - math.pi


def compute_zone_distance(
    ranges: np.ndarray,
    robot_angles: np.ndarray,
    range_min: float,
    range_max: float,
    window: ZoneWindow,
) -> Tuple[float, bool]:
    """Distancia minima valida dentro de una ventana angular.

    Retorna (distancia_metros, valido). Si no hay lecturas validas en
    la ventana, retorna (inf, False).
    """
    lo = math.radians(window.lo_deg)
    hi = math.radians(window.hi_deg)

    if lo <= hi:
        in_window = (robot_angles >= lo) & (robot_angles <= hi)
    else:
        # La ventana cruza +-180 grados.
        in_window = (robot_angles >= lo) | (robot_angles <= hi)

    finite = np.isfinite(ranges)
    in_range = (ranges >= range_min) & (ranges <= range_max)
    mask = in_window & finite & in_range

    if not np.any(mask):
        return float('inf'), False
    return float(np.min(ranges[mask])), True


def compute_all_zones(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    front_offset_rad: float,
    sign: int,
    windows: Dict[str, ZoneWindow],
) -> Dict[str, Tuple[float, bool]]:
    """Calcula la distancia minima de cada zona nombrada en ``windows``."""
    ranges_arr = np.asarray(ranges, dtype=float)
    scan_angles = _scan_angles(len(ranges_arr), angle_min, angle_increment)
    robot_angles = _robot_frame_angles(scan_angles, front_offset_rad, sign)

    result = {}
    for name, window in windows.items():
        result[name] = compute_zone_distance(
            ranges_arr, robot_angles, range_min, range_max, window
        )
    return result
