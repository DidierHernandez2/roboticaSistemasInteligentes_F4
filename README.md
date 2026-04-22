# Equipo Fantastic Four — Implementación de Robótica y Sistemas Inteligentes

<img width="331" height="496" alt="Captura de pantalla 2026-03-17 a la(s) 9 47 18 p m" src="https://github.com/user-attachments/assets/935a6e47-0d9a-458d-832b-6bb80e144825" />

Simulación del robot **Robotino 3** en Webots con ROS 2, integrando navegación autónoma, exploración por fronteras y seguimiento de personas mediante visión computacional.

---

## Paquetes principales

- **robotino_webots** — driver del robot, configuración de Nav2, mundos y launchers
- **vision** — nodos de percepción, planificación de rutas y exploración

---

## Launchers

### `frontier_explore.launch.py` — Exploración autónoma por fronteras

Lanza el robot en modo exploración: mapea el entorno de forma autónoma sin intervención humana.

**¿Qué hace?**

1. Abre Webots con el mundo `robotino_apartment.wbt`.
2. Levanta `slam_toolbox` para construir el mapa en tiempo real con el LIDAR.
3. Inicia `frontier_exploration_node` que implementa la siguiente máquina de estados:

| Estado | Descripción |
|---|---|
| `SPINNING` | Giro inicial de 360° para que slam_toolbox tenga datos suficientes |
| `FINDING` | Detecta celdas frontera (celdas libres adyacentes a zonas desconocidas) y las agrupa en clusters |
| `PATH_PLANNING` | Solicita un camino A* al centroide del cluster más lejano (para maximizar área explorada) |
| `MOVING` | Sigue el camino; `obstacle_avoidance_node` filtra `/cmd_vel` para evitar colisiones |
| `ARRIVED` | Llegó al frontier, busca el siguiente |
| `DONE` | No quedan fronteras: exploración completa |

4. Visualiza los clusters de fronteras y el objetivo actual en RViz (`frontier.rviz`).

**Nodos lanzados (secuencia temporal):**

```
t=0s   Webots
t=5s   robotino_controller
t=6s   robot_state_publisher
t=7s   slam_toolbox + static TFs
t=9s   lifecycle_manager (slam)
t=12s  path_planning_node (A*)
t=13s  obstacle_avoidance_node + laser_map_node
t=14s  frontier_exploration_node
t=15s  RViz2
```

**Cómo lanzar:**
```bash
ros2 launch robotino_webots frontier_explore.launch.py
```

---

### `yolo_person_detect.launch.py` — Seguimiento de personas con YOLO

Lanza el robot en modo seguimiento: detecta personas con una cámara RGB-D y las sigue navegando de forma autónoma.

**¿Qué hace?**

1. Abre Webots con el mundo configurado y carga un mapa pre-construido con `map_server`.
2. Levanta el stack completo de Nav2 (controller, planner, smoother, BT navigator, collision monitor) y `slam_toolbox` para localización.
3. Inicia `yolo_person_node` con la siguiente máquina de estados:

| Estado | Descripción |
|---|---|
| `SEARCHING` | El robot gira lentamente hasta detectar una persona |
| `PERSON_FOUND` | Detiene el giro, espera datos de profundidad |
| `MEASURING_DISTANCE` | Usa la máscara de segmentación YOLO + imagen de profundidad del Kinect para calcular la posición 3D de la persona y transformarla al frame `map` |
| `PATH_PLANNING` | Solicita un camino A* hacia la posición de la persona |
| `MOVING_AND_OBSTACLEDETECTION` | Publica velocidad de atracción en `/cmd_vel_desired`; `obstacle_avoidance_node` la filtra antes de enviarla al robot |
| `ARRIVED` | Llegó cerca de la persona (< 1 m), vuelve a `SEARCHING` |

4. Modela `yolo11n-seg.pt` (YOLO11 nano segmentación) sobre el tópico `/kinect_sim/rgb/image_raw`.
5. Nodos adicionales de mapeo: `map_trace_node`, `bayesian_mapper_node`, `laser_map_node`, `path_planning_node`, `potential_field_viz_node`.

**Nodos lanzados (secuencia temporal):**

```
t=0s   Webots
t=5s   robotino_controller
t=6s   robot_state_publisher
t=7s   slam_toolbox + static TFs
t=8s   map_server
t=9s   lifecycle_manager (slam)
t=12s  controller_server
t=14s  planner_server
t=15s  smoother_server
t=16s  behavior_server
t=17s  velocity_smoother
t=18s  collision_monitor + bt_navigator
t=20s  lifecycle_manager (nav2)
t=23s  path_planning_node + potential_field_viz_node
t=24s  map_trace_node + bayesian_mapper_node + laser_map_node
t=24s  RViz2
t=25s  yolo_person_node + obstacle_avoidance_node
```

**Cómo lanzar:**
```bash
ros2 launch robotino_webots yolo_person_detect.launch.py
```

> El mapa por defecto es `~/robotec_ws/map_2.yaml`. Se puede cambiar con el argumento `map_file`.

---

## Dependencias

- ROS 2 Humble
- Webots (snap)
- `slam_toolbox`, `nav2_*`
- `ultralytics` (YOLO)
- `cv_bridge`, `tf2_ros`, `tf_transformations`

---

## Estructura del workspace

```
robotec_ws/
├── src/
│   ├── robotino_webots/     # driver, config Nav2, mundos, launchers
│   └── vision/              # yolo_person_node, frontier_exploration_node,
│                            # path_planning_node, obstacle_avoidance_node, ...
├── map_2.yaml / map_2.pgm   # mapa pre-construido (apartamento)
└── map_tec_2.yaml / .pgm    # mapa alternativo
```
