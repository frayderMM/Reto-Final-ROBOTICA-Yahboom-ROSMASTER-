"""Logica de giro en cruce (chasis Ackermann), replica exacta de
``_handle_girar`` / ``_compute_turn_target`` en
``state_machine_node.py``, para validar en el simulador local antes
de portar cualquier ajuste al nodo real.

Como el chasis Ackermann no puede rotar sobre su propio eje, el giro
se aproxima con un arco de avance lento (velocidad lineal baja +
angular maxima) que cierra el lazo contra el yaw real (odometria), no
contra un tiempo fijo.
"""

import math
from dataclasses import dataclass
from typing import Tuple


def normalizar_angulo(angulo: float) -> float:
    while angulo > math.pi:
        angulo -= 2.0 * math.pi
    while angulo <= -math.pi:
        angulo += 2.0 * math.pi
    return angulo


def diferencia_angular(objetivo: float, actual: float) -> float:
    return normalizar_angulo(objetivo - actual)


def calcular_objetivo_giro(yaw_actual: float, direccion: str, angulo_deg: float = 90.0) -> float:
    """direccion en {'DERECHA', 'IZQUIERDA', 'ATRAS'}.

    ``angulo_deg`` replica ``angulo_giro_deg`` de ``state_machine_node.py``
    (default 90, subido a 95 en el robot real para compensar que el
    arco Ackermann suele quedar corto del objetivo). ATRAS siempre es
    180 fijo, no usa este valor.
    """
    angulo_rad = math.radians(angulo_deg)
    if direccion == 'DERECHA':
        delta = -angulo_rad
    elif direccion == 'IZQUIERDA':
        delta = angulo_rad
    elif direccion == 'ATRAS':
        delta = math.pi
    else:
        delta = 0.0
    return normalizar_angulo(yaw_actual + delta)


@dataclass
class ParametrosAlineacion:
    """Replica ``_handle_alinear`` de ``state_machine_node.py``: usa DOS
    puntos del lado derecho (metodo S1/S2, no el ajuste de linea) para
    corregir el heading contra la pared REAL despues de GIRAR, en vez
    de confiar solo en el angulo objetivo fijo + odometria."""
    tolerancia_m: float = 0.02
    velocidad_lineal_mps: float = 0.06
    velocidad_angular_radps: float = 0.3
    tiempo_max_s: float = 4.0


def calcular_comando_alinear(
    right_front_valido: bool, right_front_m: float,
    right_rear_valido: bool, right_rear_m: float,
    elapsed_s: float, params: ParametrosAlineacion,
) -> Tuple[float, float, bool]:
    """Retorna (linear_x, angular_z, terminado)."""
    if not (right_front_valido and right_rear_valido):
        # Sin pared derecha de referencia (p.ej. abertura tras el giro):
        # el yaw de GIRAR ya dejo al robot orientado al cardinal
        # correcto, se continua sin correccion adicional.
        return 0.0, 0.0, True

    error = right_front_m - right_rear_m
    if abs(error) <= params.tolerancia_m or elapsed_s >= params.tiempo_max_s:
        return 0.0, 0.0, True

    w = -params.velocidad_angular_radps if error > 0.0 else params.velocidad_angular_radps
    return params.velocidad_lineal_mps, w, False


@dataclass
class ParametrosGiro:
    velocidad_lineal_mps: float = 0.08
    velocidad_angular_radps: float = 0.5
    tolerancia_giro_deg: float = 4.0


def calcular_comando_giro(
    yaw_actual: float, yaw_objetivo: float, params: ParametrosGiro
) -> Tuple[float, float, bool]:
    """Retorna (linear_x, angular_z, terminado)."""
    error = diferencia_angular(yaw_objetivo, yaw_actual)
    tolerancia_rad = math.radians(params.tolerancia_giro_deg)

    if abs(error) <= tolerancia_rad:
        return 0.0, 0.0, True

    v = params.velocidad_lineal_mps
    w = params.velocidad_angular_radps if error > 0.0 else -params.velocidad_angular_radps
    return v, w, False
