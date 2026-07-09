# Diagnostico unique_line -- pegar logs aqui

Este archivo es solo para diagnosticar problemas de `unique_line` en
pista real. Se captura el log en el robot y se pega abajo (seccion
"Log capturado"); con eso reviso que sensor/estado esta fallando y
ajusto parametros en el PC (nunca se edita el YAML directo en el
robot, ver `FLUJO_DE_TRABAJO.md`).

---

## 1. Que se agrego para poder diagnosticar

`unique_line_node.py` ahora imprime (ver `_log_diagnostico`):

- **Siempre**, en cada cambio de estado: `ESTADO: <anterior> -> <nuevo>`
  con `front`, `wall_raw` (lectura cruda), `wall_f` (ya filtrada),
  `side_min`, `yaw`, `target_heading`, `v`, `w`.
- **Cada `diag_log_period_s` segundos** (1.0 por defecto, en
  `unique_line_params.yaml`) aunque no cambie de estado -- para ver si
  una lectura oscila o queda mal calibrada sin llegar a disparar una
  transicion.

## 2. Como capturar un log util

En el robot, dentro del contenedor, despues del `colcon build` +
`source install/setup.bash` de siempre:

```bash
ros2 launch capytown_granprix unique_line.launch.py 2>&1 | tee ~/unique_line_run.log
```

Deja correr el robot en la pista el tiempo suficiente para que se vea
el problema reportado (ej. el giro falso al confundir la pared con un
obstaculo), despues `Ctrl+C` para detener.

Para copiar el contenido del log (sin salir del contenedor):

```bash
cat ~/unique_line_run.log
```

Copia esa salida completa (o al menos 10-15 segundos alrededor del
momento donde el robot gira mal) y pegala en la seccion de abajo.

## 3. Que mas revisar antes de pegar el log

- **Calibracion del LiDAR** (`front_offset_deg`, `invert_left_right`):
  correr `python3 lidar_viz.py` (con `DISPLAY=:0`, ver
  `FLUJO_DE_TRABAJO.md` paso 3) y verificar que el frente/derecha/
  izquierda del dibujo coincidan con la realidad, sobre todo si el
  LiDAR se remonto o se toco algo desde la ultima calibracion (la que
  quedo documentada en `capytown_granprix/config/granprix_params.yaml`).
- **Montaje/distancia real del LiDAR al borde del robot**: si el
  sensor no esta centrado, `target_wall_dist=0.12m` puede no
  corresponder a 12 cm reales de separacion pared-robot.
- Anotar aqui abajo, antes del log, en que tramo de la pista pasa el
  problema (recto largo, esquina, la pared en U, etc.) y que tan
  seguido.

## 4. Notas de lo ya ajustado (ver README_unique_line.md para el detalle)

- `front_window_deg` angostado de `[-12,-8,-4,0,4,8,12]` a
  `[-8,-4,0,4,8]` -- la ventana ancha alcanzaba a ver la pared lateral
  seguida como si fuera un obstaculo al frente.
- Si angostar no alcanza, el siguiente paso es angostar mas (ej.
  `[-6,-3,0,3,6]`) o revisar si `target_wall_dist`/`front_blocked_dist`
  necesitan ajustarse a la geometria real del pasillo de la pista.

---

## 5. Contexto del problema (llenar antes de pegar el log)

- Tramo de la pista donde falla:
- Con que frecuencia pasa:
- Que hace mal el robot exactamente (gira sin que haya nada al frente, se acerca demasiado a la pared, oscila, etc.):

## 6. Log capturado

```text
(pegar aqui la salida de "ros2 launch capytown_granprix unique_line.launch.py")
```
