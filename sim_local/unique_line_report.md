# Reporte de simulacion -- unique_line

Lado seguido: **DERECHA**  (`follow_right=True`, `follow_left=False`)

Resultado global: **10/10 SUCCESS**, 0 colisiones, 0 timeouts.

## Pista

Polilinea de una sola pared (ver `POINTS` en `unique_line_simulator.py`), con una esquina exterior (x=2.0), una pared saliente en U (x=4.4-5.4) y una esquina interior final (x=7.6) que el robot debe completar (heading final cercano a 90 grados, no cortar la esquina).

## Resultados por prueba

| # | dist. inicial (m) | ruido | resultado | estado final | angulo final (deg) | min dist. a pared (m) | pasos | tiempo (s) |
|---:|---:|:---:|:---:|---|---:|---:|---:|---:|
| 1 | 0.08 | no | SUCCESS | FOLLOW_WALL | 87.79 | 0.0800 | 2981 | 149.1 |
| 2 | 0.10 | no | SUCCESS | FOLLOW_WALL | 87.78 | 0.1000 | 2943 | 147.2 |
| 3 | 0.12 | no | SUCCESS | FOLLOW_WALL | 87.78 | 0.1137 | 2950 | 147.5 |
| 4 | 0.14 | no | SUCCESS | FOLLOW_WALL | 87.78 | 0.1137 | 2955 | 147.8 |
| 5 | 0.16 | no | SUCCESS | FOLLOW_WALL | 87.75 | 0.1170 | 2958 | 147.9 |
| 6 | 0.18 | no | SUCCESS | FOLLOW_WALL | 87.77 | 0.1172 | 2957 | 147.8 |
| 7 | 0.20 | si | SUCCESS | FOLLOW_WALL | 87.79 | 0.0976 | 2957 | 147.8 |
| 8 | 0.24 | si | SUCCESS | FOLLOW_WALL | 87.72 | 0.0881 | 2946 | 147.3 |
| 9 | 0.28 | si | SUCCESS | FOLLOW_WALL | 87.76 | 0.0994 | 3031 | 151.6 |
| 10 | 0.32 | si | SUCCESS | FOLLOW_WALL | 87.80 | 0.0952 | 3021 | 151.1 |

## Criterios de aceptacion (seccion 14 del pedido)

- 10/10 SUCCESS: CUMPLIDO
- 0 colisiones (min dist nunca < 0.075 m): CUMPLIDO
- 0 timeouts: CUMPLIDO
- Estado final FOLLOW_WALL o CORNER_ALIGN, angulo final 80-100 deg (banda alrededor de los 88-90 deg de referencia): ver columna `resultado`.

## Archivos generados

- `unique_line_10_tests_summary.csv`: tabla resumen (una fila por prueba).
- `unique_line_10_tests.png`: grilla de trayectorias de las 10 pruebas.
- `unique_line_runs/run_XX_dYYYY.csv`: traza detallada paso a paso de cada prueba (t, x, y, yaw, state, front_dist, wall_dist, wall_dist_f, side_min, min_wall_dist).

## Parametros usados

```text
target_wall_dist = 0.12
emergency_stop_dist = 0.1
front_blocked_dist = 0.36
front_clear_dist = 0.44
lost_wall_dist = 0.34
reacquire_wall_dist = 0.28
collision_radius = 0.075
safety_side_dist = 0.092
Kp_wall = 1.0
Kp_heading = 0.98
deadband_dist = 0.024
filter_alpha = 0.24
w_limit = 0.6
v_nom = 0.145
v_align = 0.07
v_corner = 0.05
w_corner = 0.46
v_clear = 0.075
exterior_clear_dist = 0.18
lost_required = 4
clear_required = 4
stable_required = 6
blocked_required = 2
```
