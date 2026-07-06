# Distancias procesadas del LiDAR MS200, agrupadas por zona angular
# relativa al frente del robot. Publicado por lidar_processor_node.
# Todas las distancias en metros. Si una zona no tiene lecturas validas
# dentro de rango, el campo *_valid correspondiente es false y la
# distancia se reporta como +inf (no debe usarse esa lectura).

std_msgs/Header header

float32 front          # Zona frontal: pared u obstaculo al frente
float32 right_front    # S1: sensor derecho delantero (paralelismo)
float32 right          # Zona lateral derecha (distancia a la pared derecha)
float32 right_rear     # S2: sensor derecho trasero (paralelismo)
float32 left           # Zona lateral izquierda

bool front_valid
bool right_front_valid
bool right_valid
bool right_rear_valid
bool left_valid
