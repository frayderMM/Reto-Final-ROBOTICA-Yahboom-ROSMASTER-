#!/usr/bin/env python3
"""Simulador local (sin ROS2) de un OBSTACULO PERPENDICULAR clavado a
mitad de un pasillo recto (no una esquina de la grilla): la pared que
el robot sigue tiene un muro que sale hacia adentro del pasillo, y mas
alla de la punta de ese muro el espacio sigue abierto. Usa el pipeline
COMPLETO actual (AVANZAR_PARALELO -> DECIDIR -> PAUSA_GIRO -> GIRAR ->
ALINEAR), igual que ``run_sim_laberinto.py``.

IMPORTANTE: con --ancho igual a --largo-obstaculo, el obstaculo tapa
TODO el pasillo (no hay forma de esquivarlo, es indistinguible de una
pared sin salida). Para que el rodeo sea geometricamente posible,
--ancho tiene que ser mayor que --largo-obstaculo (por defecto el
doble: pasillo de 1.20m con obstaculo de 0.60m).

Uso:
    python run_sim_obstaculo.py
    python run_sim_obstaculo.py --ancho 0.90 --largo-obstaculo 0.60

Controles: cierra la ventana o Ctrl+C en la terminal para detener.
"""

import argparse
import math

import matplotlib

try:
    matplotlib.use('TkAgg')
except Exception:  # noqa: BLE001
    pass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from environment import escanear, pasillo_obstaculo_perpendicular
from robot_model import Pose, integrar
from wall_follow_control import ParametrosControl, ajustar_linea_pared, calcular_comando
from turn_control import (
    ParametrosGiro, calcular_comando_giro, calcular_objetivo_giro,
    ParametrosAlineacion, calcular_comando_alinear,
)

DT = 0.05
NUM_PUNTOS_SCAN = 452
RANGE_MAX = 4.0
RANGE_MIN = 0.03

VENT_LINEA = (-110.0, -70.0)
VENT_FRONT = (-15.0, 15.0)
VENT_LEFT = (70.0, 110.0)
VENT_RIGHT_FRONT = (-75.0, -45.0)
VENT_RIGHT_REAR = (-135.0, -105.0)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--largo-pared', type=float, default=1.80)
    p.add_argument('--ancho', type=float, default=1.20,
                    help='ancho del pasillo en la zona del obstaculo (m)')
    p.add_argument('--largo-obstaculo', type=float, default=0.60)
    p.add_argument('--distancia-objetivo', type=float, default=0.12)
    p.add_argument('--ganancia-angulo', type=float, default=2.0)
    p.add_argument('--ganancia-distancia', type=float, default=2.0)
    p.add_argument('--ganancia-heading', type=float, default=2.0)
    p.add_argument('--angular-max', type=float, default=0.6)
    p.add_argument('--velocidad', type=float, default=0.15)
    p.add_argument('--v-giro-lineal', type=float, default=0.06)
    p.add_argument('--v-giro-angular', type=float, default=0.6)
    p.add_argument('--angulo-giro', type=float, default=95.0)
    p.add_argument('--pausa-antes-girar', type=float, default=1.0)
    p.add_argument('--tolerancia-giro-deg', type=float, default=4.0)
    p.add_argument('--tolerancia-alineacion', type=float, default=0.02)
    p.add_argument('--tiempo-max-alinear', type=float, default=4.0)
    p.add_argument('--v-alinear-lineal', type=float, default=0.06)
    p.add_argument('--v-alinear-angular', type=float, default=0.3)
    p.add_argument('--umbral-frente-pared', type=float, default=0.30)
    p.add_argument('--umbral-frente-libre', type=float, default=0.35)
    p.add_argument('--umbral-lado-libre', type=float, default=0.40)
    p.add_argument('--celda-decision', type=float, default=0.30,
                    help='cada cuanto (m) revisa derecha/frente/izquierda mientras avanza')
    p.add_argument('--margen-avance', type=float, default=None,
                    help='por defecto, proporcional a --celda-decision (misma fraccion que 0.05/0.60)')
    p.add_argument('--ventana-decision', type=float, nargs=2, default=[-100.0, -80.0])
    p.add_argument('--largo-robot', type=float, default=0.24)
    p.add_argument('--ancho-robot', type=float, default=0.16)
    return p.parse_args()


def zona_min(angulos, rangos, ventana_deg, range_min=RANGE_MIN, range_max=RANGE_MAX):
    lo, hi = math.radians(ventana_deg[0]), math.radians(ventana_deg[1])
    if lo <= hi:
        mask = (angulos >= lo) & (angulos <= hi)
    else:
        mask = (angulos >= lo) | (angulos <= hi)
    mask &= np.isfinite(rangos) & (rangos >= range_min) & (rangos <= range_max)
    if not np.any(mask):
        return float('inf'), False
    return float(np.min(rangos[mask])), True


def main():
    args = parse_args()
    if args.margen_avance is None:
        args.margen_avance = args.celda_decision * (0.05 / 0.60)

    if args.largo_obstaculo >= args.ancho:
        print(f'AVISO: --largo-obstaculo ({args.largo_obstaculo}m) >= --ancho ({args.ancho}m) -- '
              f'el obstaculo tapa todo el pasillo, no hay rodeo posible.')

    params_wf = ParametrosControl(
        distancia_objetivo_m=args.distancia_objetivo, velocidad_lineal_mps=args.velocidad,
        ganancia_angulo=args.ganancia_angulo, ganancia_distancia=args.ganancia_distancia,
        ganancia_heading=args.ganancia_heading, angular_max_radps=args.angular_max,
    )
    params_giro = ParametrosGiro(
        velocidad_lineal_mps=args.v_giro_lineal, velocidad_angular_radps=args.v_giro_angular,
        tolerancia_giro_deg=args.tolerancia_giro_deg,
    )
    params_alinear = ParametrosAlineacion(
        tolerancia_m=args.tolerancia_alineacion, velocidad_lineal_mps=args.v_alinear_lineal,
        velocidad_angular_radps=args.v_alinear_angular, tiempo_max_s=args.tiempo_max_alinear,
    )

    pasillo = pasillo_obstaculo_perpendicular(
        largo_pared_m=args.largo_pared, ancho_m=args.ancho, largo_obstaculo_m=args.largo_obstaculo
    )

    pose = Pose(x=0.15, y=0.0, theta=0.0)
    heading_objetivo = None
    ultima_distancia_valida = None
    cell_start = (pose.x, pose.y)
    estado = 'AVANZAR_PARALELO'
    decision_actual = None
    giro_objetivo = None
    ultima_decision_info = ''
    pausa_giro_inicio = 0
    alinear_inicio = 0

    trayectoria_x, trayectoria_y = [pose.x], [pose.y]
    rng = np.random.default_rng(0)

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 6))

    print('Cierra la ventana o Ctrl+C para detener.')
    try:
        paso = 0
        while paso < 3000:
            angulos, rangos = escanear(
                pose.como_tupla(), pasillo,
                angle_min=-math.pi, angle_max=math.pi, num_puntos=NUM_PUNTOS_SCAN,
                range_max=RANGE_MAX, range_min=RANGE_MIN, ruido_std=0.0, rng=rng,
            )

            if estado == 'AVANZAR_PARALELO':
                ajuste = ajustar_linea_pared(angulos, rangos, *VENT_LINEA,
                                              range_min=RANGE_MIN, range_max=RANGE_MAX, min_puntos=6)
                v, w, heading_objetivo, ultima_distancia_valida = calcular_comando(
                    ajuste, pose.theta, heading_objetivo, ultima_distancia_valida, params_wf
                )
                pose = integrar(pose, v, w, DT)
                avance = math.hypot(pose.x - cell_start[0], pose.y - cell_start[1])
                front_d, front_v = zona_min(angulos, rangos, VENT_FRONT)
                frente_cerca = front_v and front_d < args.umbral_frente_pared

                if avance >= (args.celda_decision - args.margen_avance) or frente_cerca:
                    right_d, right_v = zona_min(angulos, rangos, tuple(args.ventana_decision))
                    left_d, left_v = zona_min(angulos, rangos, VENT_LEFT)
                    derecha_libre = right_v and right_d > args.umbral_lado_libre
                    frente_libre = front_v and front_d > args.umbral_frente_libre
                    izquierda_libre = left_v and left_d > args.umbral_lado_libre
                    decision_actual = ('DERECHA' if derecha_libre else
                                        ('NINGUNO' if frente_libre else
                                         ('IZQUIERDA' if izquierda_libre else 'ATRAS')))
                    ultima_decision_info = (
                        f'der={derecha_libre}({right_d*100:.0f}) frente={frente_libre}({front_d*100:.0f}) '
                        f'izq={izquierda_libre}({left_d*100:.0f}) -> {decision_actual}'
                    )
                    print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                          f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')

                    if decision_actual == 'NINGUNO':
                        cell_start = (pose.x, pose.y)
                    else:
                        giro_objetivo = calcular_objetivo_giro(
                            pose.theta, decision_actual, angulo_deg=args.angulo_giro
                        )
                        pausa_giro_inicio = paso
                        estado = 'PAUSA_GIRO'

            elif estado == 'PAUSA_GIRO':
                ajuste = None
                if (paso - pausa_giro_inicio) * DT >= args.pausa_antes_girar:
                    estado = 'GIRAR'

            elif estado == 'GIRAR':
                v, w, terminado = calcular_comando_giro(pose.theta, giro_objetivo, params_giro)
                pose = integrar(pose, v, w, DT)
                ajuste = None
                if terminado:
                    alinear_inicio = paso
                    estado = 'ALINEAR'

            elif estado == 'ALINEAR':
                rf_d, rf_v = zona_min(angulos, rangos, VENT_RIGHT_FRONT)
                rr_d, rr_v = zona_min(angulos, rangos, VENT_RIGHT_REAR)
                elapsed = (paso - alinear_inicio) * DT
                v, w, terminado = calcular_comando_alinear(
                    rf_v, rf_d, rr_v, rr_d, elapsed, params_alinear
                )
                pose = integrar(pose, v, w, DT)
                ajuste = None
                if terminado:
                    cell_start = (pose.x, pose.y)
                    heading_objetivo = None
                    ultima_distancia_valida = None
                    estado = 'AVANZAR_PARALELO'

            trayectoria_x.append(pose.x)
            trayectoria_y.append(pose.y)
            paso += 1

            if pose.x > args.largo_pared - 0.1:
                print(f'\n*** FIN DEL PASILLO en paso {paso} ***')
                break

            if paso % 2 == 0:
                _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado,
                         ultima_decision_info, trayectoria_x, trayectoria_y, args)
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    print(f'\nFin: paso={paso} estado={estado} x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
          f'theta={math.degrees(pose.theta):+.0f}')
    plt.ioff()
    plt.show()


def _dibujar_robot(ax, pose, largo, ancho):
    hl, hw = largo / 2.0, ancho / 2.0
    esquinas_local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    rot = np.array([[c, -s], [s, c]])
    esquinas = esquinas_local @ rot.T + np.array([pose.x, pose.y])
    ax.add_patch(Polygon(esquinas, closed=True, facecolor='dimgray',
                          edgecolor='black', alpha=0.9, zorder=5))
    frente_local = np.array([hl, 0.0])
    frente = frente_local @ rot.T + np.array([pose.x, pose.y])
    ax.plot([pose.x, frente[0]], [pose.y, frente[1]], color='gold', linewidth=2, zorder=6)


def _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado, decision_info, tx, ty, args):
    ax.clear()

    for seg in pasillo.segmentos:
        ax.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]], color='saddlebrown', linewidth=3)

    ax.plot(tx, ty, color='tab:blue', linewidth=1, alpha=0.6, zorder=2)
    _dibujar_robot(ax, pose, args.largo_robot, args.ancho_robot)

    lo, hi = math.radians(VENT_LINEA[0]), math.radians(VENT_LINEA[1])
    en_ventana = (angulos >= lo) & (angulos <= hi) & np.isfinite(rangos)
    if np.any(en_ventana):
        a = angulos[en_ventana]
        r = rangos[en_ventana]
        px = pose.x + r * np.cos(pose.theta + a)
        py = pose.y + r * np.sin(pose.theta + a)
        ax.scatter(px, py, s=6, color='tab:red', zorder=4)

    color_estado = {
        'AVANZAR_PARALELO': 'black', 'PAUSA_GIRO': 'firebrick',
        'GIRAR': 'purple', 'ALINEAR': 'teal',
    }.get(estado, 'black')
    ax.set_title(f'estado={estado}\n{decision_info}', fontsize=9, color=color_estado)

    ax.set_xlim(-0.2, args.largo_pared + 0.3)
    ax.set_ylim(-args.ancho - 0.2, 0.3)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.4)


if __name__ == '__main__':
    main()
