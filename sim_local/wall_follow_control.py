"""Logica de control de seguimiento de pared derecha por REGRESION DE
LINEA + Kp. Funciones puras (sin ROS2, sin estado de nodo) para poder
probarlas en el simulador local y despues portarlas tal cual a
``lidar_processor_node`` (el ajuste de linea) y ``wall_follower_node``
(el control Kp) del paquete real.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class AjusteLinea:
    angulo_rad: float       # angulo de la pared respecto al frente del robot
    distancia_m: float      # distancia perpendicular del robot a la pared
    n_puntos: int


def ajustar_linea_pared(
    angulos_robot: np.ndarray,
    rangos: np.ndarray,
    ventana_lo_deg: float,
    ventana_hi_deg: float,
    range_min: float,
    range_max: float,
    min_puntos: int = 6,
) -> Optional[AjusteLinea]:
    """Ajusta una recta y = m*x + b a los puntos del LiDAR dentro de la
    ventana angular dada (marco del robot: x=adelante, y=izquierda).

    Retorna None si no hay suficientes puntos validos para un ajuste
    confiable (equivale a "sin pared derecha de referencia").
    """
    lo = math.radians(ventana_lo_deg)
    hi = math.radians(ventana_hi_deg)

    if lo <= hi:
        en_ventana = (angulos_robot >= lo) & (angulos_robot <= hi)
    else:
        en_ventana = (angulos_robot >= lo) | (angulos_robot <= hi)

    validos = en_ventana & np.isfinite(rangos) & (rangos >= range_min) & (rangos <= range_max)
    n = int(np.sum(validos))
    if n < min_puntos:
        return None

    a = angulos_robot[validos]
    r = rangos[validos]
    x = r * np.cos(a)
    y = r * np.sin(a)

    m, b = np.polyfit(x, y, 1)
    angulo = math.atan(m)
    distancia = abs(b) / math.sqrt(m * m + 1.0)

    return AjusteLinea(angulo_rad=angulo, distancia_m=distancia, n_puntos=n)


@dataclass
class ParametrosControl:
    distancia_objetivo_m: float = 0.07
    velocidad_lineal_mps: float = 0.15
    ganancia_angulo: float = 2.0       # Kp sobre radianes (escala distinta a la version de 2 puntos)
    ganancia_distancia: float = 2.0    # Kp sobre metros
    ganancia_heading: float = 2.0      # Kp de respaldo sin pared (radianes)
    angular_max_radps: float = 0.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def calcular_comando(
    ajuste: Optional[AjusteLinea],
    yaw_actual: float,
    heading_objetivo: Optional[float],
    params: ParametrosControl,
) -> Tuple[float, float, Optional[float]]:
    """Calcula (linear_x, angular_z, nuevo_heading_objetivo).

    Si hay pared, corrige angulo Y distancia SIMULTANEAMENTE (suma
    ponderada), no alternando entre uno u otro. Alternar (corregir solo
    angulo, luego solo distancia) crea un ciclo que no se amortigua:
    cada correccion de distancia induce un error de angulo (al girar
    cambia el heading), que dispara la correccion de angulo, que vuelve
    a inducir error de distancia -- oscilacion sostenida verificada en
    sim_local/ (±1.4 cm indefinidamente, nunca se asienta). La suma
    simultanea sí converge (std < 0.01 cm en la cola del recorrido).
    Si no hay pared, mantener rumbo con Kp de heading sobre el yaw.
    """
    if ajuste is None:
        if heading_objetivo is None:
            heading_objetivo = yaw_actual
        error_heading = _angle_diff(heading_objetivo, yaw_actual)
        correccion = params.ganancia_heading * error_heading
        angular = _clamp(correccion, -params.angular_max_radps, params.angular_max_radps)
        return params.velocidad_lineal_mps, angular, heading_objetivo

    # Hay pared: se olvida el heading objetivo (se recaptura fresco la
    # proxima vez que se pierda la pared).
    heading_objetivo = None

    # Geometria: si la pared (horizontal en el mundo) se ve en el marco
    # del robot con pendiente m, entonces angulo_rad = atan(m) =
    # -theta_mundo (para un robot casi paralelo). Para corregir
    # theta_mundo -> 0 se necesita w = -k*theta_mundo = +k*angulo_rad
    # (sin signo negativo). Con el signo cambiado el lazo es de
    # realimentacion POSITIVA y el robot diverge en menos de 1 s --
    # verificado en sim_local/ antes de portar esto al robot real.
    error_distancia = params.distancia_objetivo_m - ajuste.distancia_m
    correccion = params.ganancia_angulo * ajuste.angulo_rad + params.ganancia_distancia * error_distancia

    angular = _clamp(correccion, -params.angular_max_radps, params.angular_max_radps)
    return params.velocidad_lineal_mps, angular, heading_objetivo


def _angle_diff(target: float, current: float) -> float:
    d = target - current
    while d > math.pi:
        d -= 2.0 * math.pi
    while d <= -math.pi:
        d += 2.0 * math.pi
    return d
