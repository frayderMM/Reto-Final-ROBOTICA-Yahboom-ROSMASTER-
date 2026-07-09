#!/usr/bin/env python3
"""Simulador local (sin ROS2) de la logica ``unique_line`` (seguimiento
de una sola pared con LiDAR, ver ``unique_line_control.py``).

Pista: una UNICA pared (no un pasillo con dos lados) definida como una
polilinea con una esquina exterior, una pared saliente en forma de U
(esquina interior + tope + esquina interior de salida) y una esquina
interior final. El robot la sigue por su lado derecho (o izquierdo,
segun ``UniqueLineConfig.follow_right``/``follow_left``) usando
``escanear()`` (ray casting, mismo motor que el resto de ``sim_local/``)
y la FSM de ``unique_line_control.py``.

Corre las 10 pruebas pedidas (distancia lateral inicial 0.08..0.32 m,
ruido leve en las ultimas 4) y genera:

- ``unique_line_runs/run_XX_dYYYY.csv``: traza completa por paso
  (t, x, y, yaw, state, front_dist, wall_dist, wall_dist_f, side_min,
  min_wall_dist) de cada prueba.
- ``unique_line_10_tests_summary.csv``: una fila por prueba con el
  resultado (SUCCESS/FAIL, angulo final, distancia minima a pared, etc).
- ``unique_line_10_tests.png``: grilla 2x5 con la trayectoria de las
  10 pruebas sobre la pista.
- ``unique_line_report.md``: resumen legible de los resultados.

Uso:
    python unique_line_simulator.py
    python unique_line_simulator.py --follow left
"""

import argparse
import csv
import math
import os

import matplotlib

try:
    matplotlib.use('Agg')  # generacion de PNG sin ventana interactiva
except Exception:  # noqa: BLE001
    pass

import matplotlib.pyplot as plt
import numpy as np

from environment import Pasillo
from robot_model import Pose, integrar
from unique_line_control import UniqueLineConfig, UniqueLineFSM, compute_readings

DT = 0.05
NUM_PUNTOS_SCAN = 720  # resolucion 0.5 deg, suficiente para angulos cada 2-6 deg pedidos
RANGE_MAX = 4.0
RANGE_MIN = 0.03
MAX_STEPS = 4500  # 225 s simulados de margen

_ANGULOS_ROBOT = np.linspace(-math.pi, math.pi, NUM_PUNTOS_SCAN, endpoint=False)


def _segmentos_a_arrays(pasillo: Pasillo):
    a = np.array([s.a for s in pasillo.segmentos], dtype=float)
    b = np.array([s.b for s in pasillo.segmentos], dtype=float)
    return a, b


def escanear_vectorizado(pose, seg_a, seg_b, angulos_robot, range_max, range_min, ruido_std, rng):
    """Version vectorizada (numpy, sin bucles Python) de
    ``environment.escanear``, misma semantica de ray-casting: para cada
    angulo del robot, la distancia al segmento mas cercano que corta el
    rayo dentro de [range_min, range_max), o inf si ninguno.

    El bucle doble (angulos x segmentos) de ``environment.escanear`` es
    demasiado lento para las ~30 000 lecturas de las 10 pruebas (720
    angulos x hasta 2200 pasos x 10 pruebas): esta version calcula todas
    las intersecciones angulo-segmento de una vez con broadcasting.
    """
    x, y, theta = pose
    origen = np.array([x, y], dtype=float)
    ang_mundo = theta + angulos_robot
    d = np.stack([np.cos(ang_mundo), np.sin(ang_mundo)], axis=1)  # (A,2)

    v2 = seg_b - seg_a                       # (S,2)
    v1 = origen[None, :] - seg_a             # (S,2)
    cruz_v2_v1 = v2[:, 0] * v1[:, 1] - v2[:, 1] * v1[:, 0]  # (S,)

    v3 = np.stack([-d[:, 1], d[:, 0]], axis=1)  # (A,2)

    denom = np.einsum('sk,ak->as', v2, v3)      # (A,S)
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = cruz_v2_v1[None, :] / denom
        t2 = np.einsum('sk,ak->as', v1, v3) / denom

    valido = (np.abs(denom) > 1e-9) & (t1 >= range_min) & (t2 >= 0.0) & (t2 <= 1.0)
    t1_enmascarado = np.where(valido, t1, np.inf)
    minimo = np.min(t1_enmascarado, axis=1)
    rangos = np.where(minimo < range_max, minimo, np.inf)

    if ruido_std > 0.0:
        finito = np.isfinite(rangos)
        ruido = rng.normal(0.0, ruido_std, size=rangos.shape)
        rangos = np.where(finito, np.maximum(range_min, rangos + ruido), rangos)

    return rangos

# Pista: polilinea de la UNICA pared seguida (ver seccion 14 del pedido).
POINTS = [
    (0.0, 0.0),
    (2.0, 0.0),
    (2.0, -1.0),   # esquina exterior
    (4.4, -1.0),
    (4.4, 0.2),    # pared saliente / U
    (5.4, 0.2),
    (5.4, -1.0),
    (7.6, -1.0),
    (7.6, 0.8),    # esquina interior final
]

# Meta: mas alla de la esquina interior final (x=7.6, la pared sigue
# subiendo hasta y=0.8). OJO: y por si solo NO alcanza -- el giro de la
# U (x~4.2-4.4) tambien pasa por y>=0.45 mientras el robot gira hacia el
# norte ahi (bug encontrado simulando: goal_y solo se disparaba primero
# en la U, no en la esquina final). Por eso se exige ademas x>=GOAL_X_MIN,
# solo alcanzable tras recorrer toda la pista hasta el tramo final.
GOAL_Y = 0.45
GOAL_X_MIN = 7.3

DISTANCIAS_INICIALES = [0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.24, 0.28, 0.32]
NOISE_STD = 0.005
DROPOUT_PROB = 0.012
NOISE_LAST_N = 4  # ultimas 4 pruebas de la lista, con ruido

RUNS_DIR = os.path.join(os.path.dirname(__file__), 'unique_line_runs')
SUMMARY_CSV = os.path.join(os.path.dirname(__file__), 'unique_line_10_tests_summary.csv')
SUMMARY_PNG = os.path.join(os.path.dirname(__file__), 'unique_line_10_tests.png')
REPORT_MD = os.path.join(os.path.dirname(__file__), 'unique_line_report.md')

_SUMMARY_FIELDS = [
    'prueba', 'distancia_inicial_m', 'ruido', 'resultado', 'razon',
    'estado_final', 'angulo_final_deg', 'x_final_m', 'y_final_m',
    'min_wall_dist_m', 'colision', 'pasos', 'tiempo_s',
]


def build_track() -> Pasillo:
    p = Pasillo()
    for a, b in zip(POINTS[:-1], POINTS[1:]):
        p.agregar(a[0], a[1], b[0], b[1])
    return p


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--follow', choices=['right', 'left'], default='right',
                     help='lado de la pared a seguir (la pista de prueba esta '
                          'trazada para seguimiento derecho por defecto)')
    ap.add_argument('--sin-figuras-individuales', action='store_true',
                     help='no genera PNG individual por prueba, solo la grilla resumen')
    return ap.parse_args()


def run_single_test(cfg: UniqueLineConfig, initial_dist: float, con_ruido: bool, seed: int):
    pasillo = build_track()
    seg_a, seg_b = _segmentos_a_arrays(pasillo)
    rng = np.random.default_rng(seed)

    pose = Pose(x=0.10, y=initial_dist, theta=0.0)
    fsm = UniqueLineFSM(cfg, heading_inicial=0.0)

    filas = []
    min_wall_dist_run = math.inf
    colisionado = False
    exito = False
    razon = 'timeout'
    paso = 0

    for paso in range(MAX_STEPS):
        ruido_std = NOISE_STD if con_ruido else 0.0
        angulos = _ANGULOS_ROBOT
        rangos = escanear_vectorizado(
            pose.como_tupla(), seg_a, seg_b, angulos,
            range_max=RANGE_MAX, range_min=RANGE_MIN, ruido_std=ruido_std, rng=rng,
        )
        if con_ruido and DROPOUT_PROB > 0.0:
            drop = rng.random(rangos.shape) < DROPOUT_PROB
            rangos = np.where(drop, np.inf, rangos)

        front_dist, wall_dist_raw, side_min = compute_readings(angulos, rangos, cfg)

        finitos = rangos[np.isfinite(rangos)]
        min_este_paso = float(np.min(finitos)) if finitos.size > 0 else float('inf')
        min_wall_dist_run = min(min_wall_dist_run, min_este_paso)

        v, w = fsm.step(pose.x, pose.y, pose.theta, front_dist, wall_dist_raw, side_min, DT)

        filas.append((
            round(paso * DT, 3), round(pose.x, 4), round(pose.y, 4), round(pose.theta, 4),
            fsm.state, round(front_dist, 4), round(wall_dist_raw, 4),
            round(fsm.wall_dist_f, 4) if fsm.wall_dist_f is not None else '',
            round(side_min, 4), round(min_este_paso, 4),
        ))

        if min_este_paso < cfg.collision_radius:
            colisionado = True
            razon = f'colision en paso {paso} (min={min_este_paso:.3f}m < {cfg.collision_radius}m)'
            break

        if pose.y >= GOAL_Y and pose.x >= GOAL_X_MIN:
            exito = True
            razon = 'meta alcanzada tras el giro final'
            break

        pose = integrar(pose, v, w, DT)

    final_angle_deg = math.degrees(pose.theta)
    estado_final_ok = fsm.state in ('FOLLOW_WALL', 'CORNER_ALIGN')
    angulo_ok = 80.0 <= final_angle_deg <= 100.0
    resultado = 'SUCCESS' if (exito and not colisionado and estado_final_ok and angulo_ok) else 'FAIL'
    if exito and resultado == 'FAIL':
        if not estado_final_ok:
            razon += f' (pero estado final {fsm.state} no es FOLLOW_WALL/CORNER_ALIGN)'
        elif not angulo_ok:
            razon += f' (pero angulo final {final_angle_deg:.1f} fuera de 80-100 deg)'

    resumen = dict(
        distancia_inicial_m=initial_dist,
        ruido=con_ruido,
        resultado=resultado,
        razon=razon,
        estado_final=fsm.state,
        angulo_final_deg=round(final_angle_deg, 2),
        x_final_m=round(pose.x, 3),
        y_final_m=round(pose.y, 3),
        min_wall_dist_m=round(min_wall_dist_run, 4),
        colision=colisionado,
        pasos=paso + 1,
        tiempo_s=round((paso + 1) * DT, 2),
    )
    return resumen, filas, pasillo


def _guardar_csv_detalle(path, filas):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'yaw', 'state', 'front_dist', 'wall_dist',
                    'wall_dist_f', 'side_min', 'min_wall_dist'])
        w.writerows(filas)


def _guardar_csv_resumen(path, resumenes):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=_SUMMARY_FIELDS)
        w.writeheader()
        for i, r in enumerate(resumenes, start=1):
            fila = {'prueba': i}
            fila.update(r)
            w.writerow(fila)


def _dibujar_grilla(path, pasillo, resultados_con_filas, cfg):
    fig, axes = plt.subplots(2, 5, figsize=(22, 9))
    axes = axes.flatten()

    for i, (resumen, filas) in enumerate(resultados_con_filas):
        ax = axes[i]
        for seg in pasillo.segmentos:
            ax.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]], color='saddlebrown', linewidth=3)

        xs = [f[1] for f in filas]
        ys = [f[2] for f in filas]
        color = 'tab:green' if resumen['resultado'] == 'SUCCESS' else 'tab:red'
        ax.plot(xs, ys, color=color, linewidth=1.3)
        ax.plot(xs[0], ys[0], marker='o', color='tab:blue', markersize=6, zorder=5)
        ax.plot(xs[-1], ys[-1], marker='*', color=color, markersize=12, zorder=5)

        ruido_txt = ' +ruido' if resumen['ruido'] else ''
        titulo = (
            f"#{i + 1} d0={resumen['distancia_inicial_m']:.2f}m{ruido_txt}\n"
            f"{resumen['resultado']}  ang={resumen['angulo_final_deg']:.1f}°  "
            f"min={resumen['min_wall_dist_m']:.3f}m"
        )
        ax.set_title(titulo, fontsize=9, color=color)
        ax.set_aspect('equal')
        ax.set_xlim(-0.5, 8.3)
        ax.set_ylim(-1.6, 1.3)
        ax.invert_yaxis()
        ax.tick_params(labelsize=7)

    fig.suptitle(
        f"unique_line -- 10 pruebas (follow_{'right' if cfg.follow_right else 'left'})",
        fontsize=13,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _escribir_reporte(path, resumenes, cfg):
    n_ok = sum(1 for r in resumenes if r['resultado'] == 'SUCCESS')
    n_colision = sum(1 for r in resumenes if r['colision'])
    n_timeout = sum(1 for r in resumenes if r['razon'] == 'timeout')

    lineas = []
    lineas.append('# Reporte de simulacion -- unique_line\n')
    lineas.append(
        f"Lado seguido: **{'DERECHA' if cfg.follow_right else 'IZQUIERDA'}**  "
        f"(`follow_right={cfg.follow_right}`, `follow_left={cfg.follow_left}`)\n"
    )
    lineas.append(
        f"Resultado global: **{n_ok}/10 SUCCESS**, {n_colision} colisiones, "
        f"{n_timeout} timeouts.\n"
    )
    lineas.append('## Pista\n')
    lineas.append(
        'Polilinea de una sola pared (ver `POINTS` en `unique_line_simulator.py`), '
        'con una esquina exterior (x=2.0), una pared saliente en U (x=4.4-5.4) y '
        'una esquina interior final (x=7.6) que el robot debe completar (heading '
        'final cercano a 90 grados, no cortar la esquina).\n'
    )
    lineas.append('## Resultados por prueba\n')
    lineas.append(
        '| # | dist. inicial (m) | ruido | resultado | estado final | angulo final (deg) | '
        'min dist. a pared (m) | pasos | tiempo (s) |'
    )
    lineas.append('|---:|---:|:---:|:---:|---|---:|---:|---:|---:|')
    for i, r in enumerate(resumenes, start=1):
        lineas.append(
            f"| {i} | {r['distancia_inicial_m']:.2f} | "
            f"{'si' if r['ruido'] else 'no'} | {r['resultado']} | {r['estado_final']} | "
            f"{r['angulo_final_deg']:.2f} | {r['min_wall_dist_m']:.4f} | {r['pasos']} | "
            f"{r['tiempo_s']:.1f} |"
        )
    lineas.append('')
    lineas.append('## Criterios de aceptacion (seccion 14 del pedido)\n')
    lineas.append(f"- 10/10 SUCCESS: {'CUMPLIDO' if n_ok == 10 else f'NO CUMPLIDO ({n_ok}/10)'}")
    lineas.append(f"- 0 colisiones (min dist nunca < {cfg.collision_radius} m): "
                   f"{'CUMPLIDO' if n_colision == 0 else f'NO CUMPLIDO ({n_colision} colisiones)'}")
    lineas.append(f"- 0 timeouts: {'CUMPLIDO' if n_timeout == 0 else f'NO CUMPLIDO ({n_timeout} timeouts)'}")
    lineas.append('- Estado final FOLLOW_WALL o CORNER_ALIGN, angulo final 80-100 deg '
                   '(banda alrededor de los 88-90 deg de referencia): ver columna `resultado`.')
    lineas.append('')
    lineas.append('## Archivos generados\n')
    lineas.append('- `unique_line_10_tests_summary.csv`: tabla resumen (una fila por prueba).')
    lineas.append('- `unique_line_10_tests.png`: grilla de trayectorias de las 10 pruebas.')
    lineas.append('- `unique_line_runs/run_XX_dYYYY.csv`: traza detallada paso a paso de cada prueba '
                   '(t, x, y, yaw, state, front_dist, wall_dist, wall_dist_f, side_min, min_wall_dist).')
    lineas.append('')
    lineas.append('## Parametros usados\n')
    lineas.append('```text')
    for campo in (
        'target_wall_dist', 'emergency_stop_dist', 'front_blocked_dist', 'front_clear_dist',
        'lost_wall_dist', 'reacquire_wall_dist', 'collision_radius', 'safety_side_dist',
        'Kp_wall', 'Kp_heading', 'deadband_dist', 'filter_alpha', 'w_limit',
        'v_nom', 'v_align', 'v_corner', 'w_corner', 'v_clear', 'exterior_clear_dist',
        'lost_required', 'clear_required', 'stable_required', 'blocked_required',
    ):
        lineas.append(f'{campo} = {getattr(cfg, campo)}')
    lineas.append('```')

    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lineas) + '\n')


def main():
    args = parse_args()
    cfg = UniqueLineConfig(follow_right=(args.follow == 'right'), follow_left=(args.follow == 'left'))

    resumenes = []
    resultados_con_filas = []
    pasillo = None

    for i, d0 in enumerate(DISTANCIAS_INICIALES):
        con_ruido = i >= (len(DISTANCIAS_INICIALES) - NOISE_LAST_N)
        resumen, filas, pasillo = run_single_test(cfg, d0, con_ruido, seed=1000 + i)
        resumenes.append(resumen)
        resultados_con_filas.append((resumen, filas))

        nombre = f"run_{i + 1:02d}_d{int(round(d0 * 100)):03d}cm.csv"
        _guardar_csv_detalle(os.path.join(RUNS_DIR, nombre), filas)

        ruido_txt = ' [ruido]' if con_ruido else ''
        print(
            f"Prueba {i + 1:2d} d0={d0:.2f}m{ruido_txt} -> {resumen['resultado']:7s} "
            f"estado_final={resumen['estado_final']:16s} angulo={resumen['angulo_final_deg']:7.2f}deg "
            f"min_dist={resumen['min_wall_dist_m']:.4f}m pasos={resumen['pasos']}"
        )

    _guardar_csv_resumen(SUMMARY_CSV, resumenes)
    _dibujar_grilla(SUMMARY_PNG, pasillo, resultados_con_filas, cfg)
    _escribir_reporte(REPORT_MD, resumenes, cfg)

    n_ok = sum(1 for r in resumenes if r['resultado'] == 'SUCCESS')
    print(f"\n{n_ok}/10 SUCCESS. Resumen: {SUMMARY_CSV}\nGrilla: {SUMMARY_PNG}\nReporte: {REPORT_MD}")


if __name__ == '__main__':
    main()
