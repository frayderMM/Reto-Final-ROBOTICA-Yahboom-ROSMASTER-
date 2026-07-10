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
    ParametrosGiroDinamico, calcular_comando_giro_dinamico, diferencia_angular,
)

DT = 0.05
NUM_PUNTOS_SCAN = 452
RANGE_MAX = 4.0
RANGE_MIN = 0.03

VENT_LINEA = (-110.0, -70.0)
VENT_FRONT = (-15.0, 15.0)
VENT_FRONT_ESTRECHO = (-8.0, 8.0)  # cono mas angosto para logica simple: evita confundir
                                    # una pared lateral vista en diagonal con un obstaculo real
VENT_LEFT = (70.0, 110.0)
VENT_RIGHT_FRONT = (-75.0, -45.0)   # S1, usado por ALINEAR
VENT_RIGHT_REAR = (-135.0, -105.0)  # S2, usado por ALINEAR
VENT_LINEA_ADELANTO = (-95.0, -70.0)  # ANTICIPACION: porcion mas adelantada de VENT_LINEA,
                                       # ver right_ahead_valid en lidar_processor_node.py
RANGE_MAX_PARED = 0.50  # rango propio, corto, para VENT_LINEA/VENT_LINEA_ADELANTO: sin esto,
                         # el ajuste encuentra CUALQUIER pared dentro de RANGE_MAX (4m) y nunca
                         # detecta "perdida" aunque la pared seguida (~12cm) ya haya terminado
                         # -- ver right_wall_max_range_m en lidar_processor_node.py

INICIO_THETA = -math.pi / 2.0  # mirando hacia el "norte" (Y decreciente), hacia A3/A2/A1


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--distancia-objetivo', type=float, default=0.12)
    p.add_argument('--ganancia-angulo', type=float, default=2.0)
    p.add_argument('--ganancia-distancia', type=float, default=2.0)
    p.add_argument('--ganancia-heading', type=float, default=2.0)
    p.add_argument('--angular-max', type=float, default=0.6)
    p.add_argument('--velocidad', type=float, default=0.225)  # +50% sobre 0.15
    p.add_argument('--v-giro-lineal', type=float, default=0.06)
    p.add_argument('--v-giro-angular', type=float, default=0.6)
    p.add_argument('--tolerancia-giro-deg', type=float, default=4.0)
    p.add_argument('--angulo-giro', type=float, default=90.0,
                    help='angulo objetivo de giro en grados. GIRAR_IZQUIERDA no revisa NINGUN '
                         'sensor mientras gira -- cierra el lazo solo contra el yaw de '
                         'odometria hasta llegar a este angulo (+/- --tolerancia-giro-deg), '
                         'pase lo que pase alrededor. No hay forma de interrumpirlo antes.')
    p.add_argument('--pausa-antes-girar', type=float, default=1.0,
                    help='segundos detenido entre DECIDIR y el arco de GIRAR')
    p.add_argument('--tolerancia-alineacion', type=float, default=0.02)
    p.add_argument('--tiempo-max-alinear', type=float, default=4.0)
    p.add_argument('--v-alinear-lineal', type=float, default=0.06)
    p.add_argument('--v-alinear-angular', type=float, default=0.3)
    p.add_argument('--umbral-frente-pared', type=float, default=0.40)
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
    p.add_argument('--logica', choices=['simple', 'actual'], default='simple',
                    help='simple: avanzar recto, pegarse a la pared derecha si esta a menos '
                         'de --umbral-pared-cerca, seguir recto si esta mas lejos, girar '
                         'izquierda si hay obstaculo al frente (sin PAUSA_GIRO/ALINEAR ni '
                         'decision de grilla derecha/frente/izquierda/atras). '
                         'actual: la maquina de estados completa (DECIDIR/PAUSA_GIRO/GIRAR/ALINEAR).')
    p.add_argument('--umbral-pared-cerca', type=float, default=0.30,
                    help='solo --logica simple: por debajo de esto, corrige para pegarse a la '
                         'pared derecha; por encima, sigue recto sin corregir')
    p.add_argument('--frente-confirmaciones', type=int, default=3,
                    help='solo --logica simple: ciclos seguidos con el frente bloqueado antes '
                         'de girar (evita que un vistazo diagonal de un solo ciclo dispare un '
                         'giro innecesario)')
    p.add_argument('--min-puntos-adelanto', type=int, default=4,
                    help='solo --logica simple: puntos minimos en VENT_LINEA_ADELANTO (ventana '
                         'de ANTICIPACION, mas adelantada que VENT_LINEA) para considerarla '
                         'valida -- por debajo, se frena en seco de inmediato (un solo ciclo, '
                         'sin confirmar varios) asumiendo que la pared se abrio (esquina '
                         'concava/hueco a la derecha)')
    p.add_argument('--tiempo-reanudar-avance', type=float, default=1.5,
                    help='solo --logica simple: si tras PAUSA_LINEA_PERDIDA el lado derecho NO '
                         'estaba libre, avanza recto a ciegas (ignora VENT_LINEA_ADELANTO, pero '
                         'no el frente) por este tiempo antes de rearmar la deteccion de perdida '
                         '-- si no, como no se movio, se dispararia la misma pausa de inmediato '
                         'otra vez (loop infinito de parar/verificar)')
    p.add_argument('--tiempo-verificar-hueco', type=float, default=2.0,
                    help='solo --logica simple: al confirmar perdida de la pared derecha, se '
                         'detiene este tiempo (s) y RECIEN despues verifica con distancia '
                         'puntual (no la linea) si el lado derecho esta realmente libre, antes '
                         'de comprometerse a girar')
    p.add_argument('--angulo-minimo-giro', type=float, default=45.0,
                    help='giro DINAMICO: grados minimos (por odometria, solo de resguardo) '
                         'antes de poder detectar "ya quedo paralelo" y parar')
    p.add_argument('--angulo-maximo-giro', type=float, default=150.0,
                    help='giro DINAMICO: tope de seguridad si nunca encuentra pared paralela')
    p.add_argument('--tolerancia-paralelo', type=float, default=4.0,
                    help='giro DINAMICO: grados de angulo de linea considerados "paralelo"')
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
    if args.logica == 'simple':
        _correr_logica_simple(args)
    else:
        _correr_logica_actual(args)


def _correr_logica_actual(args):
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


def _correr_logica_simple(args):
    """CUATRO reglas, sin grilla de decision ni PAUSA_GIRO/ALINEAR,
    pero con AJUSTE DE LINEA (no solo distancia puntual) para el lado
    derecho -- evita confundir una pared vista en diagonal con un
    obstaculo nuevo, porque el ajuste da angulo Y distancia en vez de
    un numero suelto:

    1. Avanzar recto mientras el frente este libre.
    2. Si hay ajuste de linea valido de la pared derecha, corregir con
       Kp (angulo + distancia hacia --distancia-objetivo) para
       mantenerse paralelo y cerca.
    3. Si se PIERDE la ventana de ANTICIPACION (VENT_LINEA_ADELANTO,
       porcion mas adelantada de VENT_LINEA -- ver --min-puntos-
       adelanto), se frena EN SECO de inmediato (un solo ciclo, sin
       confirmar varios: esta ventana mira mas hacia adelante, asi que
       adelanta la perdida antes que VENT_LINEA). Ambas ventanas usan
       RANGE_MAX_PARED (0.50m, no RANGE_MAX=4m) y el mismo ajuste de
       recta con rechazo de outliers -- sin el rango acotado, el
       ajuste encuentra cualquier pared lejana dentro de RANGE_MAX y
       "perdida de pared" casi nunca se detecta, aunque la pared
       seguida (a ~12cm) ya haya terminado. Al perderse, se detiene
       por completo (PAUSA_LINEA_PERDIDA) durante --tiempo-verificar-
       hueco (2s). RECIEN despues verifica con distancia puntual (no
       la linea) si el lado derecho esta realmente libre -- evita
       girar hacia un "hueco" que en realidad es algo demasiado cerca
       para que el LiDAR lo mida (que se ve igual que "sin pared"). Si
       esta libre, gira a la DERECHA; si no, retoma AVANZAR.
    4. Si detecta un obstaculo al frente (cono angosto, ver
       VENT_FRONT_ESTRECHO) sostenido durante --frente-confirmaciones
       ciclos seguidos, girar a la IZQUIERDA.

    Ambos giros son DINAMICOS, no un angulo fijo: siguen girando (con
    lectura de linea EN VIVO durante el giro) hasta quedar paralelos a
    la pared siguiente (angulo de linea ~0), con --angulo-minimo-giro
    de resguardo y --angulo-maximo-giro de tope de seguridad.

    Arranca paralelo a la pared inferior (fila 4, mirando al ESTE/
    derecha) en vez de mirando al norte -- coincide con la entrada
    real de A4 (lateral izquierdo, DETALLE_PISTA.md seccion 6).
    """
    inicio_x, inicio_y = 0.5 * args.celda_real, 3.5 * args.celda_real
    meta_x, meta_y = 5.5 * args.celda_real, 0.5 * args.celda_real
    umbral_meta = 0.25 * args.celda_real
    theta_inicio = 0.0  # mirando al este, paralelo a la pared inferior

    params_giro_dinamico = ParametrosGiroDinamico(
        velocidad_lineal_mps=args.v_giro_lineal,
        velocidad_angular_radps=args.v_giro_angular,
        angulo_minimo_deg=args.angulo_minimo_giro,
        angulo_maximo_deg=args.angulo_maximo_giro,
        tolerancia_paralelo_deg=args.tolerancia_paralelo,
    )

    pasillo = pasillo_laberinto_completo(celda_m=args.celda_real)

    pose = Pose(x=inicio_x, y=inicio_y, theta=theta_inicio)
    estado = 'AVANZAR'
    yaw_inicio_giro = None
    direccion_giro = 'IZQUIERDA'
    ultima_decision_info = ''
    num_giros = 0
    contador_frente = 0
    pausa_linea_perdida_inicio = None
    reanudar_avance_inicio = None

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

            if estado == 'AVANZAR':
                front_d, front_v = zona_min(angulos, rangos, VENT_FRONT_ESTRECHO)
                frente_cerca_1_ciclo = front_v and front_d < args.umbral_frente_pared
                contador_frente = contador_frente + 1 if frente_cerca_1_ciclo else 0
                frente_bloqueado = contador_frente >= args.frente_confirmaciones

                if frente_bloqueado:
                    num_giros += 1
                    contador_frente = 0
                    direccion_giro = 'IZQUIERDA'
                    yaw_inicio_giro = pose.theta
                    ultima_decision_info = f'obstaculo al frente ({front_d*100:.0f}cm) -> IZQUIERDA'
                    print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                          f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')
                    estado = 'GIRAR_IZQUIERDA'
                    ajuste = None
                elif (reanudar_avance_inicio is not None
                      and (paso - reanudar_avance_inicio) * DT < args.tiempo_reanudar_avance):
                    # Venimos de PAUSA_LINEA_PERDIDA con "lado derecho
                    # no esta libre": si volvieramos directo a chequear
                    # VENT_LINEA_ADELANTO, seguiria perdida en el mismo
                    # lugar (el robot no se movio) y se dispararia
                    # PAUSA_LINEA_PERDIDA otra vez de inmediato -- loop
                    # infinito de parar/verificar/parar sin avanzar
                    # nunca. Avanza recto a ciegas por
                    # --tiempo-reanudar-avance para salir de la zona
                    # antes de rearmar la deteccion.
                    ajuste = None
                    pose = integrar(pose, args.velocidad, 0.0, DT)
                else:
                    reanudar_avance_inicio = None
                    ajuste_adelanto = ajustar_linea_pared(
                        angulos, rangos, *VENT_LINEA_ADELANTO,
                        range_min=RANGE_MIN, range_max=RANGE_MAX_PARED,
                        min_puntos=args.min_puntos_adelanto,
                    )
                    ventana_adelanto_perdida = ajuste_adelanto is None

                    if ventana_adelanto_perdida:
                        # Ventana de ANTICIPACION perdida: frenar EN
                        # SECO de inmediato, un solo ciclo, sin
                        # confirmar varios (mira mas adelante que
                        # VENT_LINEA, asi que adelanta la perdida).
                        # Mismo ajuste con rechazo de outliers que
                        # VENT_LINEA (no un simple conteo -- cerca de
                        # una esquina, una pared perpendicular puede
                        # meter puntos que un conteo simple contaria
                        # como "pared todavia ahi"), y mismo rango
                        # acotado RANGE_MAX_PARED (sin esto, encuentra
                        # cualquier pared dentro de RANGE_MAX y nunca
                        # detecta la perdida).
                        ultima_decision_info = 'perdio la pared derecha -> detenido a verificar'
                        print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                              f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')
                        pausa_linea_perdida_inicio = paso
                        estado = 'PAUSA_LINEA_PERDIDA'
                        ajuste = None
                    else:
                        ajuste = ajustar_linea_pared(angulos, rangos, *VENT_LINEA,
                                                      range_min=RANGE_MIN, range_max=RANGE_MAX_PARED,
                                                      min_puntos=6)
                        if ajuste is None:
                            w = 0.0
                            pose = integrar(pose, args.velocidad, w, DT)
                        else:
                            error_distancia = args.distancia_objetivo - ajuste.distancia_m
                            correccion = (args.ganancia_angulo * ajuste.angulo_rad
                                          + args.ganancia_distancia * error_distancia)
                            w = max(-args.angular_max, min(args.angular_max, correccion))
                            pose = integrar(pose, args.velocidad, w, DT)

            elif estado == 'PAUSA_LINEA_PERDIDA':
                # Detenido --tiempo-verificar-hueco (2s) apenas se
                # confirma la perdida de la pared derecha, y RECIEN
                # despues chequea si el lado derecho esta realmente
                # libre (distancia puntual, no la linea) antes de
                # comprometerse a girar -- evita girar hacia un hueco
                # que en realidad no esta libre (ver
                # _handle_pausa_linea_perdida en state_machine_node.py).
                ajuste = None
                if (paso - pausa_linea_perdida_inicio) * DT >= args.tiempo_verificar_hueco:
                    right_d, right_v = zona_min(angulos, rangos, VENT_LINEA)
                    derecha_libre = right_v and right_d > args.umbral_lado_libre
                    if derecha_libre:
                        num_giros += 1
                        direccion_giro = 'DERECHA'
                        yaw_inicio_giro = pose.theta
                        ultima_decision_info = f'lado derecho libre ({right_d*100:.0f}cm) -> DERECHA'
                        print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                              f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')
                        estado = 'GIRAR_IZQUIERDA'  # mismo estado, sirve para cualquier direccion
                    else:
                        ultima_decision_info = 'lado derecho no esta libre -> retoma avance'
                        print(f'[paso {paso}] x={pose.x*100:.0f}cm y={pose.y*100:.0f}cm '
                              f'theta={math.degrees(pose.theta):+.0f} | {ultima_decision_info}')
                        reanudar_avance_inicio = paso
                        estado = 'AVANZAR'

            elif estado == 'GIRAR_IZQUIERDA':
                # Lectura de linea EN VIVO durante el giro (no ciego):
                # se usa para detectar cuando ya quedo paralelo a la
                # pared siguiente, en vez de un angulo fijo.
                ajuste = ajustar_linea_pared(angulos, rangos, *VENT_LINEA,
                                              range_min=RANGE_MIN, range_max=RANGE_MAX_PARED, min_puntos=6)
                angulo_girado = abs(diferencia_angular(pose.theta, yaw_inicio_giro))
                v, w, terminado = calcular_comando_giro_dinamico(
                    direccion_giro, angulo_girado, ajuste, params_giro_dinamico
                )
                pose = integrar(pose, v, w, DT)
                if terminado:
                    print(f'[paso {paso}] GIRO TERMINADO (paralelo) {direccion_giro} '
                          f'theta={math.degrees(pose.theta):+.1f} girado={math.degrees(angulo_girado):.0f}°')
                    estado = 'AVANZAR'

            trayectoria_x.append(pose.x)
            trayectoria_y.append(pose.y)
            paso += 1

            dist_meta = math.hypot(pose.x - meta_x, pose.y - meta_y)
            if dist_meta < umbral_meta:
                print(f'\n*** META ALCANZADA en paso {paso} ***')
                break

            if paso % args.dibujar_cada == 0:
                _dibujar(ax, pose, pasillo, angulos, rangos, ajuste, estado,
                         ultima_decision_info, None, 0, num_giros,
                         trayectoria_x, trayectoria_y, args, inicio_x, inicio_y, meta_x, meta_y)
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    print(f'\nFin: paso={paso} estado={estado} giros={num_giros} '
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
    if estado == 'AVANZAR':
        return 'Avanzando recto (logica simple)'
    if estado == 'GIRAR_IZQUIERDA':
        return f'Obstaculo al frente -> girando IZQUIERDA (objetivo {args.angulo_giro:.0f}°)'
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
        'AVANZAR': 'black', 'GIRAR_IZQUIERDA': 'purple',
        'PAUSA_LINEA_PERDIDA': 'firebrick',
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
