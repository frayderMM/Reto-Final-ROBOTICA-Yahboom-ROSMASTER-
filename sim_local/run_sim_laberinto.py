#!/usr/bin/env python3
"""Simulador local (sin ROS2) del LABERINTO COMPLETO Gran Prix
CapyTown, con coordenadas exactas de DETALLE_PISTA.md. El robot
arranca en A4 (entrada) y navega de forma REACTIVA (misma logica que
``modo_simplificado`` de ``state_machine_node.py``: AVANZAR_PARALELO
-> DECIDIR -> GIRAR -> AVANZAR_PARALELO, prioridad derecha -> frente
-> izquierda -> atras), sin conocer el mapa de antemano -- por eso no
necesariamente sigue la "ruta optima" del plano, sino la que resulte
de seguir la pared derecha.

Uso:
    python run_sim_laberinto.py
    python run_sim_laberinto.py --umbral-lado-libre 0.30
    python run_sim_laberinto.py --margen-avance -0.15

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

from environment import escanear, pasillo_laberinto_completo
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
VENT_RIGHT_FRONT = (-75.0, -45.0)   # S1, usado por ALINEAR
VENT_RIGHT_REAR = (-135.0, -105.0)  # S2, usado por ALINEAR

INICIO_THETA = -math.pi / 2.0  # mirando hacia el "norte" (Y decreciente), hacia A3/A2/A1


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--distancia-objetivo', type=float, default=0.12)
    p.add_argument('--ganancia-angulo', type=float, default=2.0)
    p.add_argument('--ganancia-distancia', type=float, default=2.0)
    p.add_argument('--ganancia-heading', type=float, default=2.0)
    p.add_argument('--angular-max', type=float, default=0.6)
    p.add_argument('--velocidad', type=float, default=0.15)
    p.add_argument('--v-giro-lineal', type=float, default=0.06)
    p.add_argument('--v-giro-angular', type=float, default=0.6)
    p.add_argument('--tolerancia-giro-deg', type=float, default=4.0)
    p.add_argument('--angulo-giro', type=float, default=95.0,
                    help='angulo objetivo de giro en grados (angulo_giro_deg en el yaml real)')
    p.add_argument('--pausa-antes-girar', type=float, default=1.0,
                    help='segundos detenido entre DECIDIR y el arco de GIRAR')
    p.add_argument('--tolerancia-alineacion', type=float, default=0.02)
    p.add_argument('--tiempo-max-alinear', type=float, default=4.0)
    p.add_argument('--v-alinear-lineal', type=float, default=0.06)
    p.add_argument('--v-alinear-angular', type=float, default=0.3)
    p.add_argument('--umbral-frente-pared', type=float, default=0.30)
    p.add_argument('--umbral-frente-libre', type=float, default=0.35)
    p.add_argument('--umbral-lado-libre', type=float, default=0.40)
    p.add_argument('--celda-real', type=float, default=0.60,
                    help='tamano de celda FISICO del laberinto (m), usado para trazar las '
                         'paredes (DETALLE_PISTA.md) y ubicar inicio/meta. NO tocar salvo '
                         'para simular una pista real distinta -- el robot (24x16cm, '
                         'PROPIEDADES_ROBOT.md) esta a escala de este valor, no del de '
                         '--celda-decision.')
    p.add_argument('--celda-decision', type=float, default=0.30,
                    help='cada cuanto (m) el robot revisa derecha/frente/izquierda mientras '
                         'avanza -- una grilla mas fina que --celda-real (12x8 celdas de '
                         '30cm en vez de 6x4 de 60cm) SOLO para el chequeo de intersecciones '
                         'y el dibujo de grilla; no cambia el ancho real de los pasillos.')
    p.add_argument('--margen-avance', type=float, default=None,
                    help='por defecto, proporcional a --celda-decision (misma fraccion que 0.05/0.60)')
    p.add_argument('--ventana-decision', type=float, nargs=2, default=[-100.0, -80.0])
    p.add_argument('--largo-robot', type=float, default=0.24)
    p.add_argument('--ancho-robot', type=float, default=0.16)
    p.add_argument('--max-pasos', type=int, default=20000)
    p.add_argument('--dibujar-cada', type=int, default=3)
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

    # Centro de A4 (inicio) y F1 (meta), en la grilla FISICA real (6x4
    # celdas de --celda-real) -- independiente de --celda-decision.
    inicio_x, inicio_y = 0.5 * args.celda_real, 3.5 * args.celda_real
    meta_x, meta_y = 5.5 * args.celda_real, 0.5 * args.celda_real
    umbral_meta = 0.25 * args.celda_real

    params_wf = ParametrosControl(
        distancia_objetivo_m=args.distancia_objetivo,
        velocidad_lineal_mps=args.velocidad,
        ganancia_angulo=args.ganancia_angulo,
        ganancia_distancia=args.ganancia_distancia,
        ganancia_heading=args.ganancia_heading,
        angular_max_radps=args.angular_max,
    )
    params_giro = ParametrosGiro(
        velocidad_lineal_mps=args.v_giro_lineal,
        velocidad_angular_radps=args.v_giro_angular,
        tolerancia_giro_deg=args.tolerancia_giro_deg,
    )
    params_alinear = ParametrosAlineacion(
        tolerancia_m=args.tolerancia_alineacion,
        velocidad_lineal_mps=args.v_alinear_lineal,
        velocidad_angular_radps=args.v_alinear_angular,
        tiempo_max_s=args.tiempo_max_alinear,
    )

    pasillo = pasillo_laberinto_completo(celda_m=args.celda_real)

    pose = Pose(x=inicio_x, y=inicio_y, theta=INICIO_THETA)
    heading_objetivo = None
    ultima_distancia_valida = None
    cell_start = (pose.x, pose.y)
    estado = 'AVANZAR_PARALELO'
    decision_actual = None
    giro_objetivo = None
    ultima_decision_info = ''
    num_celdas = 0
    num_giros = 0
    pausa_giro_inicio = 0
    alinear_inicio = 0

    trayectoria_x, trayectoria_y = [pose.x], [pose.y]
    rng = np.random.default_rng(0)

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 7))

    print('Cierra la ventana o Ctrl+C para detener.')
    try:
        paso = 0
        while paso < args.max_pasos:
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
                    num_celdas += 1
                    right_d, right_v = zona_min(angulos, rangos, tuple(args.ventana_decision))
                    left_d, left_v = zona_min(angulos, rangos, VENT_LEFT)
                    derecha_libre = right_v and right_d > args.umbral_lado_libre
                    frente_libre = front_v and front_d > args.umbral_frente_libre
                    izquierda_libre = left_v and left_d > args.umbral_lado_libre

                    if derecha_libre:
                        decision_actual = 'DERECHA'
                    elif frente_libre:
                        decision_actual = 'NINGUNO'
                    elif izquierda_libre:
                        decision_actual = 'IZQUIERDA'
                    else:
                        decision_actual = 'ATRAS'

                    ultima_decision_info = (
                        f'celda #{num_celdas}  der={derecha_libre}({right_d*100:.0f}) '
                        f'frente={frente_libre}({front_d*100:.0f}) izq={izquierda_libre}({left_d*100:.0f}) '
                        f'-> {decision_actual}'
                    )
                    print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                          f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')

                    if decision_actual == 'NINGUNO':
                        cell_start = (pose.x, pose.y)
                    else:
                        num_giros += 1
                        giro_objetivo = calcular_objetivo_giro(
                            pose.theta, decision_actual, angulo_deg=args.angulo_giro
                        )
                        pausa_giro_inicio = paso
                        estado = 'PAUSA_GIRO'

            elif estado == 'PAUSA_GIRO':
                # Robot detenido tiempo_pausa_antes_girar_s antes de
                # arrancar el arco de GIRAR (ver PAUSA_GIRO en
                # state_machine_node.py).
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
                # Corrige el heading contra la pared REAL (S1/S2) en vez
                # de confiar solo en el angulo objetivo fijo + odometria
                # de GIRAR (ver _handle_alinear en state_machine_node.py).
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

            # Meta aproximada: centro de F1.
            dist_meta = math.hypot(pose.x - meta_x, pose.y - meta_y)
            if dist_meta < umbral_meta:
                print(f'\n*** META ALCANZADA en paso {paso} ***')
                break

            if paso % args.dibujar_cada == 0:
                _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado,
                         ultima_decision_info, decision_actual, num_celdas, num_giros,
                         trayectoria_x, trayectoria_y, args, inicio_x, inicio_y, meta_x, meta_y)
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    print(f'\nFin: paso={paso} estado={estado} celdas={num_celdas} giros={num_giros} '
          f'x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm theta={math.degrees(pose.theta):+.0f}')
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


def _dibujar_grid(ax, celda_decision_m, celda_real_m):
    """Dibuja DOS grillas superpuestas: la fisica real (6x4 celdas de
    ``celda_real_m``, con las etiquetas A1..F4 -- la pista de verdad,
    DETALLE_PISTA.md) y una mas fina de ``celda_decision_m`` (lineas
    tenues) que marca cada cuanto el robot revisa intersecciones. Las
    paredes no dependen de esta grilla fina, solo el chequeo."""
    ancho_total = 6 * celda_real_m
    alto_total = 4 * celda_real_m

    n_cols_fino = max(1, round(ancho_total / celda_decision_m))
    n_rows_fino = max(1, round(alto_total / celda_decision_m))
    for col in range(n_cols_fino + 1):
        ax.axvline(col * celda_decision_m, color='whitesmoke', linewidth=0.5, zorder=0)
    for row in range(n_rows_fino + 1):
        ax.axhline(row * celda_decision_m, color='whitesmoke', linewidth=0.5, zorder=0)

    for col in range(7):
        ax.axvline(col * celda_real_m, color='lightgray', linewidth=0.8, zorder=0)
    for row in range(5):
        ax.axhline(row * celda_real_m, color='lightgray', linewidth=0.8, zorder=0)
    letras = 'ABCDEF'
    for c in range(6):
        for r in range(4):
            ax.text((c + 0.5) * celda_real_m, (r + 0.5) * celda_real_m, f'{letras[c]}{r+1}',
                     ha='center', va='center', fontsize=8, color='lightgray', zorder=0)


def _descripcion_accion(estado, decision_actual, args):
    """Frase legible de 'que esta haciendo el robot ahora mismo', para
    mostrar en el visualizador ademas del nombre tecnico del estado."""
    if estado == 'AVANZAR_PARALELO':
        return f'Avanzando — siguiendo la pared derecha a {args.distancia_objetivo*100:.0f}cm'
    if estado == 'PAUSA_GIRO':
        return f'Detenido {args.pausa_antes_girar:.0f}s antes de girar'
    if estado == 'GIRAR':
        direccion = decision_actual or '?'
        return f'Girando {direccion} (objetivo {args.angulo_giro:.0f}°, arco Ackermann)'
    if estado == 'ALINEAR':
        return 'Alineando con la pared real (corrigiendo con el LiDAR, S1/S2)'
    return estado


def _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado, decision_info,
             decision_actual, num_celdas, num_giros, tx, ty, args,
             inicio_x, inicio_y, meta_x, meta_y):
    ax.clear()

    _dibujar_grid(ax, args.celda_decision, args.celda_real)

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

    ax.plot(inicio_x, inicio_y, marker='*', markersize=14, color='tab:green', zorder=7)
    ax.plot(meta_x, meta_y, marker='*', markersize=14, color='tab:orange', zorder=7)

    color_estado = {
        'AVANZAR_PARALELO': 'black', 'PAUSA_GIRO': 'firebrick',
        'GIRAR': 'purple', 'ALINEAR': 'teal',
    }.get(estado, 'black')
    accion = _descripcion_accion(estado, decision_actual, args)
    info = f'estado={estado}  celdas={num_celdas}  giros={num_giros}\n{decision_info}'
    ax.set_title(info, fontsize=9, color=color_estado)
    ax.text(0.01, 0.99, accion, transform=ax.transAxes, ha='left', va='top',
            fontsize=12, fontweight='bold', color=color_estado,
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor=color_estado, alpha=0.9),
            zorder=10)

    margen = args.celda_real * 0.5
    ax.set_xlim(-margen, 6 * args.celda_real + margen)
    ax.set_ylim(4 * args.celda_real + margen, -margen)  # invertido: Y crece hacia abajo
    ax.set_aspect('equal')


if __name__ == '__main__':
    main()
