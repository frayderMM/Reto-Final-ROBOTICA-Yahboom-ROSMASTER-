"""Entorno 2D simple (pasillo con paredes como segmentos) y un LiDAR
simulado por ray casting. Sin dependencia de ROS2 -- solo numpy.

Convencion: marco del ROBOT ya "calibrado" (x=adelante, y=izquierda,
angulo 0=frente, +90=izquierda), igual que el LiDAR real despues de
aplicar front_offset_deg/invert_left_right. El simulador no reproduce
el problema de calibracion del LiDAR (eso ya se resolvio en el robot
real); aqui el foco es validar el algoritmo de control.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Segmento:
    a: Tuple[float, float]
    b: Tuple[float, float]


@dataclass
class Pasillo:
    """Coleccion de segmentos de pared (derecha, izquierda, lo que sea)."""

    segmentos: List[Segmento] = field(default_factory=list)

    def agregar(self, ax, ay, bx, by):
        self.segmentos.append(Segmento((ax, ay), (bx, by)))


def _interseccion_rayo_segmento(
    origen: np.ndarray, direccion: np.ndarray, seg: Segmento
) -> Optional[float]:
    """Distancia a lo largo del rayo hasta el segmento, o None si no cruza."""
    a = np.array(seg.a, dtype=float)
    b = np.array(seg.b, dtype=float)
    v1 = origen - a
    v2 = b - a
    v3 = np.array([-direccion[1], direccion[0]])

    denom = np.dot(v2, v3)
    if abs(denom) < 1e-9:
        return None

    t1 = (v2[0] * v1[1] - v2[1] * v1[0]) / denom  # producto cruz 2D / denom
    t2 = np.dot(v1, v3) / denom

    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return float(t1)
    return None


def escanear(
    pose: Tuple[float, float, float],
    pasillo: Pasillo,
    angle_min: float,
    angle_max: float,
    num_puntos: int,
    range_max: float,
    range_min: float = 0.03,
    ruido_std: float = 0.0,
    rng: Optional[np.random.Generator] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Simula un LiDAR 2D. Retorna (angulos_robot, rangos).

    ``pose`` = (x, y, theta) del robot en el marco MUNDO. Los angulos
    retornados ya estan en el marco del ROBOT (no del mundo).
    """
    x, y, theta = pose
    origen = np.array([x, y], dtype=float)
    angulos = np.linspace(angle_min, angle_max, num_puntos, endpoint=False)
    rangos = np.full(num_puntos, np.inf, dtype=float)

    if rng is None:
        rng = np.random.default_rng()

    for i, ang_robot in enumerate(angulos):
        ang_mundo = theta + ang_robot
        direccion = np.array([math.cos(ang_mundo), math.sin(ang_mundo)])

        mejor = range_max
        for seg in pasillo.segmentos:
            t = _interseccion_rayo_segmento(origen, direccion, seg)
            if t is not None and range_min <= t < mejor:
                mejor = t

        if mejor < range_max:
            if ruido_std > 0.0:
                mejor = max(range_min, mejor + rng.normal(0.0, ruido_std))
            rangos[i] = mejor
        else:
            rangos[i] = float('inf')

    return angulos, rangos


def pasillo_recto_con_quiebre(
    largo_m: float = 6.0,
    ancho_m: float = 0.60,
    offset_y0: float = 0.0,
    quiebre_x: float = 3.0,
    quiebre_delta_y: float = -0.04,
    gap_x: Optional[float] = None,
    gap_ancho: float = 0.15,
) -> Pasillo:
    """Pasillo de prueba: pared derecha con un leve quiebre de angulo a
    mitad de camino (para probar que el ajuste de linea detecta un
    angulo distinto de cero), pared izquierda recta, y opcionalmente
    un hueco en la pared derecha (para probar el fallback sin pared).
    """
    p = Pasillo()

    y_der_0 = offset_y0 - ancho_m / 2.0
    y_der_1 = y_der_0 + quiebre_delta_y

    if gap_x is None:
        p.agregar(0.0, y_der_0, quiebre_x, y_der_0)
        p.agregar(quiebre_x, y_der_0, largo_m, y_der_1)
    else:
        gap_fin = gap_x + gap_ancho
        p.agregar(0.0, y_der_0, gap_x, y_der_0)
        p.agregar(gap_fin, y_der_0, quiebre_x, y_der_0)
        p.agregar(quiebre_x, y_der_0, largo_m, y_der_1)

    y_izq = offset_y0 + ancho_m / 2.0
    p.agregar(0.0, y_izq, largo_m, y_izq)

    return p
