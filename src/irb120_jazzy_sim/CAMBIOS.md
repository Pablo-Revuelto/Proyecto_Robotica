# Cambios realizados sobre `irb120_jazzy_sim`

Documento que resume las modificaciones aplicadas al paquete base para
cumplir y ampliar el enunciado de la Práctica 3 (configuración de
MoveIt 2 para el ABB IRB-120 en ROS 2 Jazzy).

---

## 1. SRDF (`config/irb120.srdf`)

- Añadidas dos configuraciones predefinidas exigidas por el enunciado:
  - `home`  → todas las articulaciones a 0.
  - `ready` → postura de trabajo (`joint_2=-0.5`, `joint_3=0.5`, `joint_5=1.0`).
- Eliminado el `virtual_joint` redundante (la URDF ya define el enlace
  fijo `world → base_link` mediante `world_to_base`).

## 2. URDF / Xacro

- `urdf/irb120.urdf.xacro`: nuevo argumento `use_mock_hardware`.
- `urdf/irb120_macro.urdf.xacro`: el macro propaga el parámetro.
- `urdf/irb120_ros2control.xacro`: si `use_mock_hardware:=true` se carga
  el plugin `mock_components/GenericSystem` en lugar de
  `gz_ros2_control/GazeboSimSystem`. Esto permite probar MoveIt sin
  necesidad de Gazebo.

## 3. Configuración de MoveIt

- Nuevo `config/pilz_cartesian_limits.yaml` con los límites cartesianos
  para habilitar el pipeline Pilz.
- Nuevo `config/moveit_cpp.yaml` con la estructura específica que
  necesita `MoveItPy` (claves `planning_scene_monitor_options`,
  `planning_pipelines.pipeline_names`, `plan_request_params`).
- `config/moveit.rviz`: añadida la herramienta **PublishPoint**
  (publica en `/clicked_point`) y activado el marker interactivo del
  TCP (`Interactive Marker Size: 0.2`).

## 4. Launch principal `launch/moveit_irb120.launch.py`

Reescrito para arrancar de forma autónoma todo lo que pide el
enunciado:

- `robot_state_publisher`
- `ros2_control_node` con plugin mock (si `use_fake_hardware:=true`)
- Spawners de `joint_state_broadcaster` y `irb120_arm_controller`
- `move_group` con los pipelines **OMPL** y **Pilz industrial**
- `rviz2` con la `moveit_config` cargada

Argumentos: `use_fake_hardware` (def. `true`), `use_sim_time` (def. `false`).

## 5. Launch combinado `launch/full_irb120.launch.py`

Pasa explícitamente `use_fake_hardware:=false` y `use_sim_time:=true`
al incluir el MoveIt launch, para que sea Gazebo (vía
`gz_irb120.launch.py`) quien aporte el `controller_manager`.

## 6. Scripts Python con MoveItPy

- `irb120_jazzy_sim/_moveit_config.py`: módulo auxiliar que construye
  la `moveit_config` completa (URDF, SRDF, kinematics, OMPL, moveit_cpp).
- `irb120_jazzy_sim/move_to_joint.py`: ejecuta una trayectoria
  articular hardcodeada. Las posiciones se pasan como `numpy.ndarray`
  en el orden de las articulaciones del grupo `irb120_arm`.
- `irb120_jazzy_sim/move_to_clicked.py`: nodo que se suscribe a
  `/clicked_point` (topic publicado por la herramienta **Publish Point**
  de RViz) y planifica/ejecuta una trayectoria al punto seleccionado
  con orientación fija (TCP apuntando hacia abajo).
- `launch/move_to_joint.launch.py` y `launch/move_to_clicked.launch.py`:
  lanzan los scripts (la `moveit_config` la construye el propio script).
- `setup.py`: entry points `move_to_joint` y `move_to_clicked`.

## 7. Obstáculo y evitación de colisiones

- `irb120_jazzy_sim/add_obstacle.py`: nodo que publica una caja
  (`CollisionObject` de 0.15 × 0.15 × 0.4 m en `x=0.45, y=0, z=0.20`
  respecto a `world`) en `/planning_scene` con QoS *transient local*
  y republicación durante 10 s.
- `launch/demo_obstacle.launch.py`: lanza MoveIt + RViz + el nodo del
  obstáculo en un único comando.
- `launch/demo_full.launch.py`: lanza MoveIt + RViz + obstáculo +
  escucha de `/clicked_point` en un único comando (fusión D + E).
- `setup.py`: entry point `add_obstacle`.

---

# Guía paso a paso de uso

## Compilación (única vez tras los cambios)

Terminal:
```bash
cd ~/Robotica_inteligente/ros2_ws
colcon build --packages-select irb120_jazzy_sim --symlink-install
source install/setup.bash
```

> **Importante:** en cada terminal nueva ejecuta primero
> `source ~/Robotica_inteligente/ros2_ws/install/setup.bash` antes
> de lanzar nada.

## A. Demo MoveIt + RViz sin Gazebo

Una única terminal:
```bash
ros2 launch irb120_jazzy_sim moveit_irb120.launch.py
```

En RViz, panel **MotionPlanning** (parte inferior izquierda):

**Trayectoria articular (joint goal):**
1. Pestaña **Planning**.
2. En **Goal State**, despliega el menú y elige `home`, `ready`,
   `all_zero` o `<random valid>`.
3. Pulsa **Plan**, después **Execute** (o **Plan & Execute**).

**Trayectoria cartesiana usando el marker interactivo del TCP:**
1. En la barra de herramientas superior de RViz, selecciona
   la herramienta **Interact** (icono de la mano/flechas).
2. En la escena 3D aparecen unas flechas y anillos de colores sobre
   el efector `tool0` del robot fantasma naranja
   (estado objetivo). Si no aparecen, en el árbol de la izquierda:
   `MotionPlanning → Planning Request → Interactive Marker Size`,
   sube el valor (p. ej. 0.2). También confirma que
   `Query Goal State` está marcado.
3. Arrastra las flechas (traslación) y los anillos (rotación) para
   colocar el TCP en la pose objetivo.
4. Pulsa **Plan** y luego **Execute** en el panel MotionPlanning.

**Trayectoria cartesiana lineal (interpolación en línea recta):**
1. Coloca el TCP objetivo con el marker interactivo como arriba.
2. En el panel MotionPlanning, marca la casilla **Use Cartesian Path**
   (pestaña *Planning*).
3. Pulsa **Plan**. La trayectoria mostrada será una recta entre el
   estado actual y el objetivo (en lugar de la curva habitual de OMPL).
4. Pulsa **Execute**.

**Cambiar de pipeline (OMPL ↔ Pilz industrial motion planner):**
1. En el panel **MotionPlanning**, pestaña **Context**.
2. **Planning Library / Pipeline:** selecciona
   `pilz_industrial_motion_planner` (u `ompl`).
3. **Planner ID:** elige `PTP`, `LIN` o `CIRC` (solo para Pilz).
4. Vuelve a la pestaña **Planning** y pulsa **Plan & Execute**.

## B. Demo completa con Gazebo

Una única terminal:
```bash
ros2 launch irb120_jazzy_sim full_irb120.launch.py
```

Esto arranca Gazebo + el robot simulado + MoveIt + RViz. La
planificación en RViz mueve también el robot en Gazebo.

## C. Trayectoria articular vía script Python

Terminal A — MoveIt:
```bash
ros2 launch irb120_jazzy_sim moveit_irb120.launch.py
```

Terminal B — script:
```bash
ros2 launch irb120_jazzy_sim move_to_joint.launch.py
```
o equivalentemente:
```bash
ros2 run irb120_jazzy_sim move_to_joint
```

## D. Planificación a un punto clicado con el ratón

Terminal A — MoveIt:
```bash
ros2 launch irb120_jazzy_sim moveit_irb120.launch.py
```

Terminal B — escucha de clics:
```bash
ros2 launch irb120_jazzy_sim move_to_clicked.launch.py
```

En RViz: barra de herramientas superior → **Publish Point** → clic
en el espacio 3D (suelo o cualquier objeto). El robot planifica y
ejecuta hacia esa posición.

## E. Demo de evitación de colisiones (obstáculo)

Una única terminal:
```bash
ros2 launch irb120_jazzy_sim demo_obstacle.launch.py
```

En RViz aparece la caja verde como obstáculo. Desde **MotionPlanning**
planifica de `home` a `ready` (o a cualquier pose detrás de la caja):
la trayectoria rodeará el obstáculo. Si lo eliminas desde el panel
*Scene Objects* y vuelves a planificar, la trayectoria pasará por
donde estaba la caja.

## F. Demo combinada: obstáculo + click-to-plan

Una única terminal:
```bash
ros2 launch irb120_jazzy_sim demo_full.launch.py
```

Esto arranca a la vez: MoveIt + RViz + el obstáculo (caja verde) +
la escucha del topic `/clicked_point`. En RViz:

- Barra de herramientas superior → **Publish Point** → clic en el
  espacio 3D. El robot planifica y ejecuta hacia el punto evitando
  la caja.
- Alternativamente, puedes usar el panel MotionPlanning con
  `home`/`ready` y verás que las trayectorias rodean el obstáculo.
