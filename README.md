# Reto Final ROBÓTICA — Yahboom ROSMASTER R2 — Gran Prix CapyTown

Navegación autónoma por **seguimiento de pared derecha** para el laberinto
Gran Prix CapyTown (pista 360×240 cm, rejilla 6×4 celdas de 60×60 cm,
inicio en **A4**, meta en **F1**). Robot Yahboom ROSMASTER R2 (chasis
Ackermann) con Raspberry Pi 5, ROS 2 Humble, LiDAR MS200 y cámara opcional.

Documentos de referencia del reto (en esta misma carpeta):
`logica_pared_derecha_robot.md`, `DETALLE RETO 3.md`, `DETALLE_PISTA.md`,
`PROPIEDADES_ROBOT.md`, `FLUJO_DE_TRABAJO.md`.

---

## 1. Estructura del proyecto

```text
Reto Final/
├── capytown_interfaces/          # Mensajes personalizados (ament_cmake)
│   └── msg/
│       ├── LidarZones.msg        # Distancias por zona (frente/S1/derecha/S2/izquierda)
│       └── RobotEvent.msg        # Eventos discretos para métricas
│
├── capytown_granprix/             # Paquete principal de navegación (ament_python)
│   ├── capytown_granprix/
│   │   ├── geometry_utils.py      # yaw, ángulos, clamp
│   │   ├── lidar_utils.py         # extracción de zonas del /scan
│   │   ├── grid_map.py            # celda/heading por conteo de movimientos
│   │   ├── event_types.py         # constantes de RobotEvent.tipo
│   │   ├── lidar_processor_node.py    # (2) Nodo de lectura LiDAR
│   │   ├── wall_follower_node.py      # (3) Wall following derecha
│   │   ├── state_machine_node.py      # (4) Decisión de intersecciones + FSM
│   │   ├── stop_sign_detector_node.py # (5) Cámara: detección de PARE
│   │   └── metrics_logger_node.py     # (6) Métricas -> CSV
│   ├── config/granprix_params.yaml    # (7) Umbrales y parámetros
│   ├── launch/granprix_bringup.launch.py  # (8) Launch file
│   └── package.xml / setup.py / setup.cfg
│
└── README.md                      # (9) este archivo
```

### Arquitectura de nodos y tópicos

```text
/scan ──> lidar_processor_node ──> /lidar_zones ──┬──> wall_follower_node ──> /wall_follow/cmd_vel_suggestion ──┐
                                                    └──> state_machine_node <─────────────────────────────────┤
/odom_raw ─────────────────────────────────────────────────────────────────> state_machine_node               │
/camera/image_raw ──> stop_sign_detector_node ──> /pare_detectado ─────────> state_machine_node               │
                                                                                     │                          │
                                                                                     ├──> /cmd_vel  (único publicador)
                                                                                     ├──> /robot_state (String, debug)
                                                                                     └──> /robot_event (RobotEvent)
                                                                                              │
                                                                                              └──> metrics_logger_node ──> metricas_granprix.csv
```

**Regla de diseño clave:** `state_machine_node` es el **único** nodo que
escribe en `/cmd_vel`. `wall_follower_node` solo publica una *sugerencia*
de velocidad; `state_machine_node` la reenvía mientras el estado es
`AVANZAR_PARALELO` y calcula sus propios comandos en el resto de estados
(girar, alinear, detenerse). Esto evita que dos nodos manden comandos de
movimiento contradictorios al mismo tiempo.

---

## 2. Máquina de estados

```text
INICIAR
  │
  ▼
AVANZAR_PARALELO  ◄─────────────────────────────┐
  │ (avanza 60 cm manteniendo pared derecha)      │
  ▼                                               │
DETECTAR_CRUCE (detenido, confirma D/F/I)         │
  ▼                                               │
BUSCAR_PARE (cámara, hasta 3 s si detecta PARE)   │
  ▼                                               │
DECIDIR (derecha→frente→izquierda→180°)           │
  ▼                                               │
GIRAR (arco lento + yaw de odometría) ── (recto) ─┤
  ▼                                               │
ALINEAR (empareja S1≈S2 con la pared derecha)     │
  ▼                                               │
VERIFICAR_META ────────── no es meta ─────────────┘
  │
  ▼ (celda == F1)
META (fin)
```

Estado adicional `DETENIDO`: red de seguridad si se supera
`max_celdas_recorridas` sin llegar a la meta (evita loops infinitos por
fallas de sensor). No forma parte del flujo pedido, solo protege la corrida.

### Por qué no se pega a la pared ni va en diagonal

- La distancia objetivo a la pared derecha es **20 cm** (rango aceptable
  18–25 cm), usando el **promedio** de dos zonas del LiDAR (S1 delantera y
  S2 trasera del lado derecho), no una distancia mínima "pegada".
- Antes de corregir distancia, el robot corrige **paralelismo** (S1≈S2).
  Esto es lo que evita el zigzag/diagonal descrito en
  `logica_pared_derecha_robot.md`.
- Los giros de intersección solo ocurren con el robot **detenido primero**
  (frenado explícito antes de `GIRAR`) y se **alinea** después de cada giro
  antes de volver a avanzar.

### Adaptación por chasis Ackermann

El ROSMASTER R2 tiene dirección Ackermann: **no puede rotar sobre su
propio eje** (radio de giro cero), a diferencia de un robot diferencial.
El documento de referencia asume motores independientes izquierda/derecha;
aquí se adapta así:

- Los giros (`GIRAR`) y la alineación fina (`ALINEAR`) se hacen con un
  **arco de avance muy lento** (`velocidad_giro_lineal_mps` /
  `velocidad_alineacion_lineal_mps`, ambas bajas) combinado con la
  velocidad angular máxima permitida, cerrando el lazo con el **yaw real
  de `/odom_raw`** (no con un tiempo fijo), hasta alcanzar el error
  angular objetivo (90°, 180° o el ángulo de alineación).
- Esta aproximación necesita espacio libre alrededor (el pasillo de 60 cm
  y la separación de ~20-30 cm a las paredes que ya mantiene el robot dan
  margen suficiente), pero **debe calibrarse en pista** — ver sección 5.

---

## 3. Instalación

Se asume el flujo de trabajo de `FLUJO_DE_TRABAJO.md`: editar en el PC,
subir a GitHub, compilar en el robot dentro del contenedor Docker.

### En el robot (`ssh root@10.42.0.1` → `docker exec -it friendly_pike bash`)

```bash
cd /root/yahboomcar_ws/src
git clone https://github.com/frayderMM/Reto-Final-ROBOTICA-Yahboom-ROSMASTER-.git reto-final
cd /root/yahboomcar_ws
colcon build --packages-select capytown_interfaces capytown_granprix
source install/setup.bash
```

> `capytown_interfaces` se debe compilar antes que `capytown_granprix`
> (colcon respeta esta dependencia automáticamente por `package.xml`, pero
> si se compila con `--packages-select` asegúrate de listar ambos).

Dependencias del sistema usadas por los nodos (ya deberían estar en la
imagen del robot; si falta alguna):

```bash
sudo apt install ros-humble-cv-bridge python3-opencv
pip3 install numpy
```

### Verificar tópicos del robot antes de lanzar

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom_raw
ros2 topic info /camera/image_raw   # si se va a usar la cámara
```

Si el bringup del robot (`capytown_esan bringup.launch.py` u otro) no
está corriendo, lanzarlo primero — este proyecto **no** reemplaza al
driver del robot, solo consume sus tópicos (`/scan`, `/odom_raw`,
`/cmd_vel`, cámara).

---

## 4. Ejecución

### Ronda 1 — Exploración (con cámara)

```bash
ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=1 usar_camara:=true
```

### Ronda 2 — Time Attack

```bash
ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=2
```

### Sin cámara (si no está disponible o para pruebas de solo LiDAR)

```bash
ros2 launch capytown_granprix granprix_bringup.launch.py usar_camara:=false
```

### Verificar en vivo

```bash
ros2 topic echo /lidar_zones
ros2 topic echo /robot_state
ros2 topic echo /robot_event
ros2 topic echo /pare_detectado
```

Al llegar a la meta (o agotar `max_celdas_recorridas`), se escribe una
fila en `~/capytown_resultados/metricas_granprix.csv` (ruta configurable
con el parámetro `csv_path` de `metrics_logger`). Formato exacto en
`DETALLE RETO 3.md`, sección 12.

> **`pare_falsos`** no se puede medir de forma 100% automática: el robot
> no tiene forma de saber si una detección fue realmente una señal PARE
> real de la pista sin comparar contra el video/observación humana.
> Revisar manualmente los eventos `PARE_DETECTADO` contra las señales
> reales de la pista y ajustar ese campo en el CSV si corresponde.

---

## 5. Calibración

Seguir el orden sugerido por `logica_pared_derecha_robot.md` (sección 20),
probando por partes antes de correr el laberinto completo.

### 5.1 Escala del odómetro (`state_machine` — `factor_dist_odom` / `factor_ang_odom`)

El `/odom_raw` del ROSMASTER R2 sobreestima tanto distancia como ángulo de
forma consistente (no es ruido aleatorio, es un factor de escala fijo).
Calibrar esto primero: si la odometría miente sobre cuánto avanzó o giró
el robot, ninguna otra calibración (avance de 60 cm, giro de 90°) va a
dar buenos resultados aunque el control esté bien ajustado.

1. Con el robot quieto, leer la pose:
   ```bash
   ros2 topic echo /odom_raw --once
   ```
2. Empujar el robot **a mano** una distancia real conocida en línea recta
   (por ejemplo 60 cm, medida con cinta métrica) y volver a leer. Calcular
   `distancia_odom = sqrt((x2-x1)^2 + (y2-y1)^2)`.
3. Con el robot quieto de nuevo, anotar el quaternion de orientación,
   girarlo **a mano** un ángulo real conocido (90°, ayudándose de una
   escuadra) sin trasladarlo, y volver a leer. El yaw (para un quaternion
   con x=y=0) es `yaw = 2*atan2(z, w)`; calcular `angulo_odom = yaw2 - yaw1`.
4. Calcular los factores de corrección:
   ```text
   factor_dist_odom = distancia_real / distancia_odom
   factor_ang_odom  = angulo_real / angulo_odom
   ```
5. Poner esos valores en `granprix_params.yaml`, dentro de `state_machine`:
   ```yaml
   factor_dist_odom: 0.9474   # ejemplo calibrado: avance real 76 cm / odometro 78.3 cm
   factor_ang_odom: 0.9899    # ejemplo calibrado: giro real 90° / odometro 90.92°
   ```
   `state_machine_node` aplica estos factores a `/odom_raw` apenas lo
   recibe (`_on_odom`), así que tanto el avance por celda (60 cm) como el
   cierre de los giros por yaw quedan corregidos automáticamente — no hace
   falta tocar `wall_follower` ni `lidar_processor` para esto.
6. Repetir la prueba 2-3 veces (avance y giro) para confirmar que el
   factor es estable; si varía mucho entre pruebas, sospechar de
   deslizamiento de ruedas más que de un error de escala fijo.

### 5.2 Orientación y sentido del LiDAR (`lidar_processor`)

1. Con el robot quieto frente a una pared, correr:
   ```bash
   ros2 run capytown_granprix lidar_processor_node
   ros2 topic echo /lidar_zones
   ```
2. Si `front` no baja al acercar un obstáculo al frente real del robot,
   ajustar `front_offset_deg` en `granprix_params.yaml` (probar 180 si el
   LiDAR está montado invertido).
3. Si al acercar un obstáculo por la **derecha** física el valor que baja
   es `left` (o viceversa), poner `invert_left_right: true`.
4. Repetir hasta que `front`, `right`/`right_front`/`right_rear` y `left`
   correspondan físicamente a lo esperado.

### 5.3 Avance recto y distancia a pared (`wall_follower`)

1. Colocar el robot en un pasillo recto de 60 cm, pared a la derecha.
2. `ros2 run capytown_granprix wall_follower_node` y observar
   `/wall_follow/cmd_vel_suggestion`.
3. Ajustar `distancia_objetivo_m`, `ganancia_angulo`, `ganancia_distancia`
   y `angular_max_radps` hasta que el robot recorra ~60 cm sin desviarse
   y sin zigzaguear (si oscila, bajar ganancias; si corrige muy lento,
   subirlas).

### 5.4 Giro de 90° (`state_machine`, estado GIRAR)

1. Probar en una intersección real o simulando espacio libre a un lado.
2. Ajustar `velocidad_giro_lineal_mps`, `velocidad_giro_angular_radps` y
   `tolerancia_giro_deg` hasta lograr un giro de ~90° con error menor a
   ±3-4°, sin chocar contra las paredes del cruce.
3. El radio aproximado del arco es `r ≈ velocidad_giro_lineal_mps /
   velocidad_giro_angular_radps` (con los valores por defecto, 0.08/0.5 ≈
   0.16 m). Un radio pequeño es indispensable para que el giro de 180°
   (callejón sin salida) quepa dentro de una celda de 60 cm: si el robot
   se sale del pasillo al girar, bajar la velocidad lineal y/o subir la
   angular para reducir el radio, no al revés.

### 5.5 Alineación tras el giro (`ALINEAR`)

Ajustar `tolerancia_alineacion_m`, `velocidad_alineacion_lineal_mps` y
`velocidad_alineacion_angular_radps` hasta que, tras un giro, el robot
quede con S1≈S2 (diferencia menor a 2-3 cm) en un tiempo razonable
(menor a `tiempo_max_alinear_s`).

### 5.6 Intersección completa

Verificar que en un cruce real el robot: se detiene, confirma
correctamente derecha/frente/izquierda (`muestras_confirmacion`,
`consenso_minimo`), decide, gira si corresponde, se alinea y retoma el
avance recto — sin quedar en diagonal.

### 5.7 Señal de PARE (`stop_sign_detector`)

1. Colocar una señal de PARE (roja) frente a la cámara y correr:
   ```bash
   ros2 run capytown_granprix stop_sign_detector_node
   ros2 topic echo /pare_detectado
   ```
2. Si no detecta o detecta de más, ajustar `rango1_min/max`,
   `rango2_min/max` (segmentación HSV de rojo), `area_minima_px`,
   `area_maxima_px` y la relación de aspecto. Activar
   `publicar_debug: true` y ver `/pare_detectado/debug_image` en RViz o
   `rqt_image_view` (el robot tiene TigerVNC con entorno gráfico) para
   ver el recuadro detectado.
3. Ajustar `frames_confirmacion` (más alto = menos falsos positivos, más
   lento para reaccionar) y `frames_perdida`.

### 5.8 Umbrales de decisión

`umbral_frente_pared_m`, `umbral_frente_libre_m`, `umbral_lado_libre_m` y
`umbral_colision_m` en `state_machine` son los que definen "pared cerca",
"camino libre" y "colisión". Ajustar según el ancho real del pasillo
(60 cm) y el tamaño del robot (24×16 cm).

---

## 6. Notas de diseño y limitaciones conocidas

- **Localización por conteo de celdas:** no hay marcas físicas de meta ni
  ArUco; la celda actual se estima contando avances de 60 cm y giros
  (`grid_map.py`), asumiendo ejecución sin deslizamiento severo. Si el
  robot pierde tracción o se desalinea mucho, la estimación de celda
  puede desincronizarse del mapa real.
- **Colisión:** no hay tópico confirmado de parachoques/bumper en el
  bringup del robot (ver `PROPIEDADES_ROBOT.md`); se usa el LiDAR frontal
  como proxy (`umbral_colision_m`) para contar colisiones y hacer un
  retroceso corto de seguridad.
- **Pasillo sin pared derecha:** si tras un giro no hay pared derecha de
  referencia, `ALINEAR` se salta (el `yaw` de `GIRAR` ya dejó al robot
  orientado al cardinal correcto) — ver `logica_pared_derecha_robot.md`
  sección 15.
- **Ronda 2 (time attack):** este proyecto no implementa memoria de ruta
  óptima entre rondas (BFS/A\*); ambas rondas usan la misma lógica
  reactiva de pared derecha. La mejora de la ronda 2 vendría de afinar
  la calibración (velocidades más altas, giros más precisos), no de
  planificación de ruta.

---

## 7. Flujo de trabajo (PC ↔ robot)

Ver `FLUJO_DE_TRABAJO.md` para el ciclo completo de edición en PC → commit
→ push → pull y build en el robot. Resumen:

```bash
# PC
git add .
git commit -m "mensaje"
git push origin main

# Robot (dentro del contenedor)
cd /root/yahboomcar_ws/src/reto-final
git fetch origin && git reset --hard origin/main
cd /root/yahboomcar_ws
colcon build --packages-select capytown_interfaces capytown_granprix
source install/setup.bash
ros2 launch capytown_granprix granprix_bringup.launch.py
```
