# unique_line -- seguimiento de una sola pared con LiDAR

Rama `unique_line`. Logica de navegacion reactiva, independiente del
resto del reto (Gran Prix CapyTown / laberinto): el robot sigue **una
sola pared lateral** (derecha o izquierda, configurable) usando LiDAR,
evita obstaculos frontales, toma esquinas interiores y exteriores sin
cortar en diagonal, y puede bordear una pared saliente en forma de U.

Pensada para Yahboom ROSMASTER R2 (chasis Ackermann, no rota sobre su
eje), ROS 2 Humble, LiDAR en `/scan`, comando en `/cmd_vel`.

## 1. Idea central

Un seguidor de pared reactivo simple ("girar hasta que el frente este
libre") corta las esquinas en diagonal porque gira mientras sigue
detectando la pared vieja. `unique_line` lo evita con una maquina de
estados de 6 estados:

| Estado | Que hace |
|---|---|
| `FOLLOW_WALL` | Avanza recto, corrige con Kp hacia `target_wall_dist` (angulo + distancia sumados, sin alternar). Los giros donde la pared se curva "hacia afuera" (ej. avanzar por una esquina normal) los resuelve solo este control continuo, sin cambiar de estado. |
| `INTERIOR_TURN_90` | Frente bloqueado (esquina interior, pared saliente, fondo de corredor) -> gira 90° **alejandose** de la pared seguida, detenido si hay riesgo de colision. |
| `EXTERIOR_CLEAR` | La pared lateral desaparecio y el frente sigue libre (esquina exterior) -> avanza recto un tramo corto (`exterior_clear_dist`) ANTES de girar, para no cortar la esquina. |
| `EXTERIOR_TURN_90` | Gira 90° **hacia** la pared seguida, para recuperarla del otro lado de la esquina exterior. |
| `CORNER_ALIGN` | Tras cualquier giro, estabiliza contra la pared real (o mantiene rumbo si todavia no aparece) antes de volver a `FOLLOW_WALL`. |
| `EMERGENCY_STOP` | Prioridad maxima en cualquier estado: frente demasiado cerca -> parar y esperar varias lecturas seguras antes de reanudar. |

Ver el docstring de cada estado en
[`sim_local/unique_line_control.py`](sim_local/unique_line_control.py)
para el detalle exacto de las condiciones de transicion.

## 2. Archivos

| Archivo | Rol |
|---|---|
| [`sim_local/unique_line_control.py`](sim_local/unique_line_control.py) | Logica pura (FSM + extraccion de sectores LiDAR), sin ROS2. Fuente de verdad de la logica. |
| [`sim_local/unique_line_simulator.py`](sim_local/unique_line_simulator.py) | Simulador (ray casting + cinematica de uniciclo) que valida la logica contra la pista pedida antes de portarla al robot. |
| [`sim_local/unique_line_10_tests_summary.csv`](sim_local/unique_line_10_tests_summary.csv) | Resultado resumido de las 10 pruebas. |
| [`sim_local/unique_line_10_tests.png`](sim_local/unique_line_10_tests.png) | Grilla con las 10 trayectorias. |
| [`sim_local/unique_line_report.md`](sim_local/unique_line_report.md) | Reporte detallado de la simulacion. |
| [`sim_local/unique_line_runs/`](sim_local/unique_line_runs/) | Traza paso a paso de cada una de las 10 pruebas (CSV). |
| [`capytown_granprix/capytown_granprix/unique_line_node.py`](capytown_granprix/capytown_granprix/unique_line_node.py) | Nodo ROS2: misma FSM portada, entrada `/scan` + `/odom_raw`, salida `/cmd_vel`. |
| [`capytown_granprix/config/unique_line_params.yaml`](capytown_granprix/config/unique_line_params.yaml) | Parametros (mismos valores validados en el simulador). |
| [`capytown_granprix/launch/unique_line.launch.py`](capytown_granprix/launch/unique_line.launch.py) | Launch file. |

La logica del nodo ROS2 es una copia exacta (mismos nombres de
parametros, misma FSM) de `unique_line_control.py` -- solo cambia la
capa de entrada/salida (LaserScan/Odometry en vez de ray-casting
simulado). Mismo patron que el resto del paquete (`wall_follow_control.py`
+ `wall_follower_node.py`, `turn_control.py` + `state_machine_node.py`).

## 3. Simulador: como correrlo

```bash
cd sim_local
python unique_line_simulator.py            # sigue la pared derecha (default)
python unique_line_simulator.py --follow left
```

Corre las 10 pruebas pedidas: distancia lateral inicial al arrancar de
0.08 a 0.32 m, con ruido leve (`noise_std=0.005`, `dropout_prob=0.012`)
en las ultimas 4. Pista: una sola pared (no un pasillo con dos lados)
con una esquina exterior, una pared saliente en U y una esquina
interior final -- ver `POINTS` en `unique_line_simulator.py`.

### Resultado obtenido (parametros por defecto, seguimiento derecho)

**10/10 SUCCESS, 0 colisiones, 0 timeouts.** Angulo final entre 87.72°
y 87.80° en las 10 pruebas (banda de referencia pedida: 88-90°, ver
`sim_local/unique_line_report.md` para la tabla completa fila por
fila). Distancia minima a cualquier pared siempre >= 0.080 m (limite
de colision configurado: 0.075 m).

## 4. Nodo ROS2: como correrlo

```bash
ros2 launch capytown_granprix unique_line.launch.py
# o, para seguir la pared izquierda:
ros2 launch capytown_granprix unique_line.launch.py follow_right:=false follow_left:=true
```

o directo:

```bash
ros2 run capytown_granprix unique_line_node --ros-args --params-file \
    install/capytown_granprix/share/capytown_granprix/config/unique_line_params.yaml
```

No depende de `lidar_processor_node` ni de `state_machine_node`: lee
`/scan` directo y publica `/cmd_vel` el solo. **No correr junto con
`state_machine_node`** (los dos escribirian en `/cmd_vel` a la vez).

El heading inicial se captura del primer mensaje de `/odom_raw` al
arrancar (no asume que el robot mira "al norte" del mundo): el modulo
es autonomo, no conoce el mapa.

## 5. Parametros principales

Ver `capytown_granprix/config/unique_line_params.yaml` para la lista
completa comentada. Los mas relevantes para calibrar en pista real:

- `follow_right` / `follow_left`: lado a seguir (mutuamente
  excluyentes, `assert` en `UniqueLineConfig.__post_init__`).
- `target_wall_dist` (0.12 m), `front_blocked_dist` (0.36 m),
  `front_clear_dist` (0.44 m): distancias que disparan las
  transiciones de estado.
- `v_nom`, `v_corner`, `v_align`, `v_clear`: velocidades por estado.
- `Kp_wall`, `Kp_heading`, `deadband_dist`, `filter_alpha`: control y
  filtrado de la lectura lateral.
- `front_offset_deg` / `invert_left_right`: calibracion de montaje del
  LiDAR -- mismos valores ya validados para este robot en
  `granprix_params.yaml` (ver `logica_pared_derecha_robot.md` seccion
  10 y el historial de calibracion en `granprix_params.yaml`).

## 6. Notas de diseno

- Sin flags booleanos de "lectura valida": `front_dist`/`wall_dist`/
  `side_min` son simplemente distancias: si un sector no tiene ningun
  rayo valido dentro de su tolerancia angular, se reporta como
  `range_max_m` (4.0 m por defecto) -- "lejos" ya significa "no hay
  pared ahi", sin necesitar un campo aparte.
- Los sectores del LiDAR (`front_window`, `wall_window`,
  `wide_wall_window`) son **angulos discretos** (ej.
  `[-12,-8,-4,0,4,8,12]`), no una ventana continua: se toma el rayo
  real mas cercano a cada angulo pedido (tolerancia `sector_tol_deg`).
- `EMERGENCY_STOP` tiene prioridad maxima y se evalua en cualquier
  estado antes de despachar la logica normal, igual que la regla de
  seguridad global de `state_machine_node.py` en el resto del reto.
