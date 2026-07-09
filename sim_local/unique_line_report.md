# Reporte de simulacion -- unique_line

Lado seguido: **DERECHA**  (`follow_right=True`, `follow_left=False`)

Resultado global: **10/10 SUCCESS**, 0 colisiones, 0 timeouts.

## Pista

Polilinea de una sola pared (ver `POINTS` en `unique_line_simulator.py`), con una esquina exterior (x=2.0), una pared saliente en U (x=4.4-5.4) y una esquina interior final (x=7.6) que el robot debe completar (heading final cercano a 90 grados, no cortar la esquina).

## Resultados por prueba

| # | dist. inicial (m) | ruido | resultado | estado final | angulo final (deg) | min dist. a pared (m) | pasos | tiempo (s) |
|---:|---:|:---:|:---:|---|---:|---:|---:|---:|
| 1 | 0.08 | no | SUCCESS | FOLLOW_WALL | 88.05 | 0.0800 | 2382 | 119.1 |
| 2 | 0.10 | no | SUCCESS | FOLLOW_WALL | 88.00 | 0.1000 | 2294 | 114.7 |
| 3 | 0.12 | no | SUCCESS | FOLLOW_WALL | 88.05 | 0.1021 | 2306 | 115.3 |
| 4 | 0.14 | no | SUCCESS | FOLLOW_WALL | 88.01 | 0.1013 | 2302 | 115.1 |
| 5 | 0.16 | no | SUCCESS | FOLLOW_WALL | 88.02 | 0.1004 | 2351 | 117.5 |
| 6 | 0.18 | no | SUCCESS | FOLLOW_WALL | 88.01 | 0.1013 | 2350 | 117.5 |
| 7 | 0.20 | si | SUCCESS | FOLLOW_WALL | 88.00 | 0.0858 | 2358 | 117.9 |
| 8 | 0.24 | si | SUCCESS | FOLLOW_WALL | 87.96 | 0.0840 | 2376 | 118.8 |
| 9 | 0.28 | si | SUCCESS | FOLLOW_WALL | 87.91 | 0.0868 | 2361 | 118.0 |
| 10 | 0.32 | si | SUCCESS | FOLLOW_WALL | 87.92 | 0.0859 | 2365 | 118.2 |

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
Kp_wall = 1.6
Kp_heading = 1.5
deadband_dist = 0.024
filter_alpha = 0.24
w_limit = 0.85
v_nom = 0.19
v_align = 0.09
v_corner = 0.065
w_corner = 0.55
v_clear = 0.095
exterior_clear_dist = 0.18
lost_required = 4
clear_required = 4
stable_required = 6
blocked_required = 2
```
