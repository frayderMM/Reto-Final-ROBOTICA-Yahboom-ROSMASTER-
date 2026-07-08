#!/usr/bin/env python3
"""Dibuja el mapa del laberinto Gran Prix CapyTown (sin robot, sin
recorrido) en una ventana interactiva, usando las paredes exactas de
``environment.pasillo_laberinto_completo()`` (coordenadas de
DETALLE_PISTA.md).

Uso:
    python dibujar_mapa.py
"""

import matplotlib

try:
    matplotlib.use('TkAgg')
except Exception:  # noqa: BLE001
    pass

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

from environment import pasillo_laberinto_completo

COLOR_FONDO = '#f4f6f8'
COLOR_CELDA = '#e8ebef'
COLOR_GRID = '#c7ccd4'
COLOR_PARED = '#b5651d'
COLOR_TEXTO = '#3a4048'
COLOR_TEXTO_CELDA = '#9aa2ad'
COLOR_INICIO = '#2f9e44'
COLOR_META = '#ffa94d'
COLOR_META_FILL = '#fff4e6'

CELDA_M = 0.60
COLS, ROWS = 6, 4
LETRAS = 'ABCDEF'


def main():
    pasillo = pasillo_laberinto_completo()

    fig, ax = plt.subplots(figsize=(10, 7.2))
    fig.patch.set_facecolor(COLOR_FONDO)
    ax.set_facecolor(COLOR_FONDO)

    for c in range(COLS):
        for r in range(ROWS):
            ax.add_patch(mpatches.Rectangle(
                (c * CELDA_M, r * CELDA_M), CELDA_M, CELDA_M,
                facecolor=COLOR_CELDA, edgecolor='none', zorder=0,
            ))

    ax.add_patch(mpatches.Rectangle(
        (5 * CELDA_M, 0 * CELDA_M), CELDA_M, CELDA_M,
        facecolor=COLOR_META_FILL, edgecolor='none', zorder=1,
    ))

    for col in range(COLS + 1):
        ax.plot([col * CELDA_M, col * CELDA_M], [0, ROWS * CELDA_M],
                 color=COLOR_GRID, linewidth=0.8, linestyle=(0, (4, 3)), zorder=2)
    for row in range(ROWS + 1):
        ax.plot([0, COLS * CELDA_M], [row * CELDA_M, row * CELDA_M],
                 color=COLOR_GRID, linewidth=0.8, linestyle=(0, (4, 3)), zorder=2)

    for c in range(COLS):
        for r in range(ROWS):
            ax.text((c + 0.5) * CELDA_M, (r + 0.12) * CELDA_M, f'{LETRAS[c]}{r + 1}',
                     ha='center', va='center', fontsize=9, color=COLOR_TEXTO_CELDA,
                     fontweight='bold', zorder=3)

    for seg in pasillo.segmentos:
        ax.plot([seg.a[0], seg.b[0]], [seg.a[1], seg.b[1]],
                 color=COLOR_PARED, linewidth=5, solid_capstyle='round', zorder=4)

    inicio_x, inicio_y = 0.30, 2.10
    ax.plot(inicio_x, inicio_y, marker='o', markersize=20, color=COLOR_INICIO, zorder=6)
    ax.plot(inicio_x, inicio_y, marker='>', markersize=9, color='white', zorder=7)
    ax.text(inicio_x, inicio_y - 0.14, 'INICIO', ha='center', va='top', fontsize=9,
            color=COLOR_INICIO, fontweight='bold', zorder=6)

    ax.text(5.5 * CELDA_M, 0.5 * CELDA_M, 'META', ha='center', va='center',
            fontsize=13, color='#d9730d', fontweight='bold', zorder=6)

    ax.text(0, -0.34, 'Laberinto Gran Prix CapyTown — sim_local',
            fontsize=17, fontweight='bold', color=COLOR_TEXTO, ha='left', va='bottom')
    ax.text(0, -0.20, 'Reconstruido de DETALLE_PISTA.md · 360×240cm · grilla 6×4 de 60×60cm',
            fontsize=10, color=COLOR_TEXTO_CELDA, ha='left', va='bottom')

    leyenda = [
        Line2D([0], [0], color=COLOR_PARED, linewidth=4, label='Pared'),
        Line2D([0], [0], marker='o', color=COLOR_INICIO, linestyle='None', markersize=10, label='Inicio (A4)'),
        Line2D([0], [0], marker='s', color=COLOR_META_FILL, markeredgecolor=COLOR_META,
               linestyle='None', markersize=10, label='Meta (F1)'),
    ]
    ax.legend(handles=leyenda, loc='upper left', bbox_to_anchor=(1.0, 1.0),
              frameon=False, fontsize=10)

    ax.set_xlim(-0.15, COLS * CELDA_M + 1.0)
    ax.set_ylim(ROWS * CELDA_M + 0.45, -0.45)
    ax.set_aspect('equal')
    ax.axis('off')

    fig.tight_layout()
    print('Cierra la ventana para terminar.')
    plt.show()


if __name__ == '__main__':
    main()
