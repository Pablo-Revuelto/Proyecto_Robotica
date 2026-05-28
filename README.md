# irb120_jazzy_sim — Simulación del ABB IRB120 en ROS 2 Jazzy

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-blue)]()
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)]()
[![MoveIt](https://img.shields.io/badge/MoveIt-2%20(moveit__py)-green)]()
[![Build](https://img.shields.io/badge/build-ament__python-lightgrey)]()

Paquete ROS 2 que implementa una **simulación de alta fidelidad** del manipulador
industrial **ABB IRB120** sobre **ROS 2 Jazzy Jalisco**, utilizando **Gazebo Harmonic**
como motor de física y **MoveIt 2** (a través de la API oficial `moveit_py`) como
*stack* de planificación de trayectorias.

El paquete integra todos los componentes necesarios para llevar el robot desde su
modelo URDF/Xacro hasta la ejecución de trayectorias programáticas:

- Modelo cinemático y visual (URDF/Xacro, mallas `.dae`/`.stl`).
- *Hardware interface* mediante `gz_ros2_control` (plugin nativo de Gazebo Sim/Harmonic).
- Controladores `ros2_control` (`joint_state_broadcaster`, `joint_trajectory_controller`).
- Configuración completa de MoveIt 2 (SRDF, cinemática, OMPL, límites, controladores).
- Lanzadores modulares (`gz_irb120`, `moveit_irb120`, `full_irb120`).
- Nodo Python de ejemplo (`move_to_joint`) con planificación articular.

---

## Tabla de contenidos

1. [Requisitos del sistema](#1-requisitos-del-sistema)
2. [Instalación de dependencias (Jazzy)](#2-instalación-de-dependencias-jazzy)
3. [Obtención y compilación del paquete](#3-obtención-y-compilación-del-paquete)
4. [Estructura del paquete](#4-estructura-del-paquete)
5. [Validación del URDF](#5-validación-del-urdf)
6. [Ejecución paso a paso](#6-ejecución-paso-a-paso)
7. [Nodo de control programático (`move_to_joint`)](#7-nodo-de-control-programático-move_to_joint)
8. [Resolución de problemas (*Troubleshooting*)](#8-resolución-de-problemas-troubleshooting)
9. [Notas sobre el informe técnico](#9-notas-sobre-el-informe-técnico)

---

## 1. Requisitos del sistema

| Componente            | Versión recomendada                |
| --------------------- | ---------------------------------- |
| Sistema operativo     | Ubuntu **24.04 LTS** (Noble Numbat) |
| Distribución ROS 2    | **Jazzy Jalisco**                  |
| Simulador             | **Gazebo Harmonic** (`gz sim 8`)   |
| MoveIt                | **MoveIt 2** (binarios de Jazzy)   |
| Python                | 3.12                               |
| Compilador            | `colcon` + `ament_python`          |

> **Importante:** La pareja oficial *ROS 2 Jazzy ⇄ Gazebo Harmonic* se integra
> mediante el meta-paquete `ros_gz` (no usar `gazebo_ros_pkgs` ni plugins de
> Gazebo Classic, son **incompatibles**).

Asegúrese de tener ROS 2 Jazzy instalado y *sourced* (por ejemplo añadiendo
`source /opt/ros/jazzy/setup.bash` a `~/.bashrc`).

---

## 2. Instalación de dependencias (Jazzy)

> ⚠️ **Aviso:** El informe técnico que acompaña a este proyecto lista algunos
> nombres de paquetes APT que **no existen** o no son los recomendados para
> Jazzy. La lista de abajo está **verificada y corregida** para `apt` en
> Ubuntu 24.04 + ROS 2 Jazzy.

### 2.1. Actualizar el índice de paquetes

```bash
sudo apt update
```

### 2.2. *Stack* de MoveIt 2 y control

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-py \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller
```

> En Jazzy, **`moveit_py` se distribuye como el binario `ros-jazzy-moveit-py`**
> y forma parte del *stack* oficial de MoveIt 2. La metapaquete `ros-jazzy-moveit`
> arrastra la mayoría de plugins (OMPL, kinematics, planning interface, etc.).

### 2.3. *Stack* de Gazebo Harmonic + bridge `ros_gz`

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control
```

> `ros-jazzy-ros-gz` ya arrastra a `ros-gz-sim`, `ros-gz-bridge` y `ros-gz-interfaces`,
> pero se listan de forma explícita para mayor claridad. El binario para el
> *hardware interface* dentro de Gazebo Sim es `ros-jazzy-gz-ros2-control`
> (no `ros-jazzy-gazebo-ros2-control`, que pertenece a Gazebo Classic y **no
> funciona** con Harmonic).

### 2.4. URDF, visualización y utilidades

```bash
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

> El paquete `ros-jazzy-tf-transformations` mencionado en el informe **no es
> necesario** para este proyecto (no se usa en ningún nodo). Si en algún
> momento lo necesita, instálelo aparte.

### 2.5. Inicialización de `rosdep` (solo la primera vez)

```bash
sudo rosdep init   # puede fallar con "sources list already exists" → es normal, continúe
rosdep update
```

> Si `rosdep init` devuelve `ERROR: default sources list file already exists`, no es un
> error real: `rosdep` ya estaba configurado de una sesión anterior. Ejecute igualmente
> `rosdep update` para actualizar la caché.

---

## 3. Obtención y compilación del paquete

Se asume que el *workspace* `ros2_ws` ya existe con el paquete dentro de
`ros2_ws/src/irb120_jazzy_sim`. Si parte desde cero:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# coloque aquí el paquete irb120_jazzy_sim (clon, copia o submódulo)
```

### 3.1. Resolución automática de dependencias

```bash
cd ~/Robotica_inteligente/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Es normal recibir `Cannot locate rosdep definition for [ament_python]`. Se trata de
> una dependencia de *buildtool* que rosdep no resuelve via APT; el propio `colcon`
> la gestiona. Si al final aparece `#All required rosdeps installed successfully`,
> todo está correcto.

### 3.2. Compilación

```bash
cd ~/Robotica_inteligente/ros2_ws
colcon build --symlink-install
```

> El *flag* `--symlink-install` permite editar `.py`, `.yaml`, `.xacro` y
> demás recursos *in place* sin necesidad de recompilar (excepto cambios en
> `setup.py` o nuevas instalaciones de ficheros).

### 3.3. *Sourcing* del entorno

```bash
source ~/Robotica_inteligente/ros2_ws/install/setup.bash
```

> Añada esta línea a `~/.bashrc` si va a trabajar habitualmente con el paquete:
> ```bash
> echo "source ~/Robotica_inteligente/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 4. Estructura del paquete

```
irb120_jazzy_sim/
├── config/                       # YAMLs de MoveIt y ros2_control
│   ├── irb120.srdf
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── ompl_planning.yaml
│   └── ros2_controllers.yaml     # update_rate: 250 Hz
├── irb120_jazzy_sim/             # Paquete Python
│   ├── __init__.py
│   └── move_to_joint.py          # Nodo demo con moveit_py
├── launch/
│   ├── gz_irb120.launch.py       # Gazebo + spawner + controladores
│   ├── moveit_irb120.launch.py   # move_group + RViz
│   └── full_irb120.launch.py     # Orquestador (gz + moveit con TimerAction)
├── meshes/
│   ├── visual/                   # .dae
│   └── collision/                # .stl
├── urdf/
│   ├── abb_resources/            # materiales y colores compartidos
│   ├── irb120.urdf.xacro         # punto de entrada
│   ├── irb120_macro.urdf.xacro
│   ├── irb120_ros2control.xacro  # gz_ros2_control/GazeboSimSystem
│   └── irb120_transmission.xacro
├── worlds/
│   └── irb120_world.sdf
├── package.xml
└── setup.py
```

---

## 5. Validación del URDF

Antes del primer lanzamiento conviene verificar que Xacro genera un URDF
válido y que los plugins de Gazebo Sim están correctamente referenciados:

```bash
ros2 run xacro xacro \
  ~/Robotica_inteligente/ros2_ws/src/irb120_jazzy_sim/urdf/irb120.urdf.xacro \
  use_gazebo:=true > /tmp/irb120.urdf

grep -E "gz_ros2_control|joint_1|package://" /tmp/irb120.urdf
```

Debe encontrar al menos las siguientes cadenas:

- `gz_ros2_control/GazeboSimSystem` *(plugin del hardware interface)*
- `gz_ros2_control-system` *(plugin SDF cargado por Gazebo Sim)*
- `joint_1` … `joint_6`
- `package://irb120_jazzy_sim/meshes/...`

---

## 6. Ejecución paso a paso

### 6.0. OBLIGATORIO — Limpieza previa, aislamiento y sourcing

> **Causa raíz de los problemas más comunes:** este *workspace* o el sistema
> pueden conservar **procesos huérfanos** (un `robot_state_publisher` zombi
> publicando un URDF antiguo en `/robot_description`, una instancia de Gazebo
> de otra sesión, etc.). En ese caso, cuando lance esta simulación, `ros_gz_sim
> create -topic robot_description` puede leer el URDF del proyecto antiguo
> (p. ej. un robot "cobot") en lugar del IRB120, y el plugin
> `gz_ros2_control-system` no se carga → no hay `controller_manager`. Esta
> sección elimina ese estado y aísla la sesión por completo.

#### Paso A — Limpieza exhaustiva de procesos residuales

Ejecute **una sola vez** (en cualquier terminal) antes de empezar:

```bash
pkill -9 -f robot_state_publisher ; \
pkill -9 -f "gz sim"              ; \
pkill -9 -f gzserver              ; \
pkill -9 -f ruby                  ; \
pkill -9 -f move_group            ; \
pkill -9 -f rviz2                 ; \
pkill -9 -f controller_manager    ; \
pkill -9 -f spawner               ; \
pkill -9 -f parameter_bridge      ; \
pkill -9 -f ros2_daemon           ; \
sleep 1
```

Verifique que no queda nada:

```bash
ps aux | grep -iE "(robot_state|gz sim|move_group|rviz2|controller_manager)" | grep -v grep
```

Debe **no devolver ninguna línea**. Si alguna persiste, anote el PID y mátela con
`kill -9 <PID>`.

> ⚠️ **`pkill -f "gz sim"` por sí solo NO basta.** El proceso problemático más
> habitual es `robot_state_publisher` huérfano (lo deja un `ros2 launch`
> anterior interrumpido con Ctrl+C). Su padre pasa a ser `init` (PID 1) y
> sobrevive a sesiones de terminal. La línea `pkill -9 -f robot_state_publisher`
> es la que lo elimina.

#### Paso B — Sourcing y aislamiento en CADA terminal nueva

En **cada** terminal (1, 2, 3 y 4) lo primero que debe ejecutar es:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robotica_inteligente/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=irb120_sim
```

- `ROS_DOMAIN_ID=42` aísla esta sesión ROS 2 de cualquier otro proceso ROS en la
  misma máquina (la red DDS solo verá nodos con el **mismo** dominio).
- `GZ_PARTITION=irb120_sim` aísla el bus gz-transport (independiente de DDS) para
  que ningún Gazebo de otro proyecto interfiera con éste.

> **Sobre el alias `actualizar`** del curso (`source install/setup.bash`):
> sigue siendo válido **siempre que el workspace contenga únicamente el
> paquete `irb120_jazzy_sim`**. Si en algún momento añade aquí paquetes de
> otro proyecto (p. ej. `cobot_*`), `source install/setup.bash` los cargará
> todos y reaparecerán las interferencias. Mantenga este workspace limpio o
> use un workspace dedicado.

> **Sobre el alias `ai-on`** del curso: fija `ROS_LOCALHOST_ONLY=1` y
> `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Es compatible con este paquete,
> pero combine con `ROS_DOMAIN_ID=42` (arriba) para el aislamiento completo.

---

### 6.1. Terminal 1 — Física + controladores

```bash
ros2 launch irb120_jazzy_sim gz_irb120.launch.py
```

Carga Gazebo Harmonic con el mundo `irb120_world.sdf`, publica el
`robot_description`, hace *spawn* del IRB120 y activa los controladores:

- `joint_state_broadcaster`
- `irb120_arm_controller` (FollowJointTrajectory, 250 Hz)

**Salida esperada:**
```
[ros_gz_sim] Entity creation successful
[spawner_joint_state_broadcaster] Configured and activated joint_state_broadcaster
[spawner_irb120_arm_controller] Configured and activated irb120_arm_controller
```

> El plugin `gz_ros2_control-system` crea internamente el nodo
> `/controller_manager`. Si los spawners no lo contactan en ~30 s, revise la
> sección de Troubleshooting.

### 6.2. Terminal 2 — Verificación de la cadena de control

```bash
ros2 control list_controllers
```

Resultado esperado (ambos en estado `active`):

```
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster  active
irb120_arm_controller        joint_trajectory_controller/JointTrajectoryController  active
```

Si el comando se queda esperando, compruebe que el nodo existe:

```bash
ros2 node list | grep controller_manager
```

### 6.3. Terminal 3 — Planificación con MoveIt 2

> **WSL2 (Windows):** RViz requiere un servidor de display. Con WSLg (Windows 11)
> funciona directamente. En Windows 10 instale VcXsrv y ejecute primero:
> `export DISPLAY=:0`. Si RViz cierra al instante, es un problema de display,
> no del paquete.

```bash
ros2 launch irb120_jazzy_sim moveit_irb120.launch.py
```

Lanza `move_group` con la configuración completa de MoveIt 2 (SRDF, cinemática
KDL, OMPL, límites articulares) y abre RViz cargando automáticamente la
configuración `config/moveit.rviz` (Fixed Frame = `base_link`, panel
*MotionPlanning* preconfigurado con el grupo `irb120_arm` y el `RobotModel`
suscrito a `/robot_description`).

En el panel *MotionPlanning* puede arrastrar el *interactive marker* del *end
effector* y pulsar **Plan & Execute**.

Las advertencias `Cannot infer URDF from ...` son **benignas**: `MoveItConfigsBuilder`
intenta inferir el fichero automáticamente pero cae al *fallback* explícito
configurado en el launcher. El planificador carga correctamente igualmente.

> Si abre RViz con la configuración por defecto (sin `-d`) verá el error
> `Frame [map] does not exist` porque el `Fixed Frame` por defecto es `map`
> y este robot no publica esa TF. La rviz config incluida usa `base_link`,
> que es publicado por `robot_state_publisher` desde el primer instante.

### 6.4. Terminal 4 — Prueba programática

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robotica_inteligente/ros2_ws/install/setup.bash
ros2 run irb120_jazzy_sim move_to_joint
```

> El `source` es obligatorio si esta terminal no hereda el entorno de otras. Sin él
> obtendrá `Package 'irb120_jazzy_sim' not found`.

Planifica y ejecuta la trayectoria articular definida en `move_to_joint.py`.
Salida esperada en consola: `Trayectoria ejecutada`.

### 6.5. Alternativa: arranque todo-en-uno

```bash
ros2 launch irb120_jazzy_sim full_irb120.launch.py
```

Incluye `gz_irb120.launch.py` + `moveit_irb120.launch.py` con un `TimerAction`
de 6 s para garantizar que el `controller_manager` esté activo antes de que
`move_group` intente conectarse a él.

---

## 7. Nodo de control programático (`move_to_joint`)

El nodo `move_to_joint.py` (instalado como ejecutable a través de
`entry_points` en `setup.py`) ilustra el patrón canónico de planificación con
`moveit_py`:

```python
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def main():
    rclpy.init()
    moveit = MoveItPy(node_name="irb120_moveit_py")
    arm = moveit.get_planning_component("irb120_arm")

    robot_state = RobotState(moveit.get_robot_model())
    robot_state.set_joint_group_positions("irb120_arm", {
        "joint_1": 0.0, "joint_2": -0.7, "joint_3": 0.9,
        "joint_4": 0.0, "joint_5":  1.0, "joint_6": 0.0,
    })

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=robot_state)

    plan = arm.plan()
    if plan:
        moveit.execute(plan.trajectory)
```

> **Nota:** `MoveItPy` requiere que `move_group` esté ejecutándose
> previamente (terminal 3). De lo contrario el nodo se quedará esperando
> los servicios del *planning pipeline*.

---

## 8. Resolución de problemas (*Troubleshooting*)

| Síntoma | Causa probable | Solución |
| ------- | -------------- | -------- |
| **Aparece un modelo extraño ("cobot", móvil con lidar, etc.) en Gazebo en lugar del IRB120** | Un `robot_state_publisher` **huérfano** de otra sesión sigue publicando un URDF antiguo en `/robot_description`. `ros_gz_sim create -topic robot_description` recoge ese URDF y *spawnea* el robot equivocado. | Aplicar el **Paso A** de la sección 6.0 (incluye `pkill -9 -f robot_state_publisher`). Verificar con `ps aux \| grep robot_state_publisher` que no queda ninguno antes de relanzar. |
| `controller_manager` no arranca / `spawner: waiting for /controller_manager/list_controllers` indefinido | El plugin `gz_ros2_control-system` no se cargó porque el URDF *spawneado* no era el del IRB120 (consecuencia del problema anterior) — o el plugin no está instalado. | 1) Aplicar **Paso A** de 6.0. 2) Verificar `ros2 node list \| grep controller`. 3) Si sigue fallando: `sudo apt install ros-jazzy-gz-ros2-control`. |
| RViz muestra `Frame [map] does not exist` o un grid vacío | RViz arrancó con la configuración por defecto (`Fixed Frame = map`). | El launcher `moveit_irb120.launch.py` carga **automáticamente** `config/moveit.rviz` con `Fixed Frame = base_link` y panel *MotionPlanning*. Si abre RViz manualmente, pase `-d $(ros2 pkg prefix irb120_jazzy_sim)/share/irb120_jazzy_sim/config/moveit.rviz`. |
| `Package 'irb120_jazzy_sim' not found` al ejecutar `ros2 run` | Terminal sin `source` del workspace. | Repetir el **Paso B** de 6.0 (`source /opt/ros/jazzy/setup.bash` + `source ~/Robotica_inteligente/ros2_ws/install/setup.bash`). |
| `Semantic description is not specified for the same robot as the URDF` | SRDF declaraba `<robot name="name">` en lugar de `<robot name="irb120">`. | **Ya corregido** en `config/irb120.srdf`. |
| RViz se cierra inmediatamente | Sin servidor de display en WSL2. | WSL2 en Windows 11: usa WSLg (automático). Windows 10: instala VcXsrv y ejecuta `export DISPLAY=:0`. |
| `Could not find plugin gz_ros2_control-system` | Paquete no instalado. | `sudo apt install ros-jazzy-gz-ros2-control` |
| `MoveItPy` queda colgado | `move_group` no está corriendo. | Lanzar primero `moveit_irb120.launch.py` y esperar confirmación `MoveGroup context initialization complete`. |
| Mallas no se renderizan en RViz/Gazebo | URIs `file://` heredadas. | Cambiar todas las rutas a `package://irb120_jazzy_sim/meshes/...`. |
| Nodos no se ven entre terminales (`ros2 node list` no muestra `controller_manager`) | Cada terminal tiene un `ROS_DOMAIN_ID` distinto. | Asegúrese de que **todas** las terminales exportan el **mismo** `ROS_DOMAIN_ID=42` (Paso B de 6.0). |

---

## 9. Notas sobre el informe técnico

Este `README.md` se ha redactado **alineado con el informe técnico
adjunto** (`informe tecnico.pdf`) pero **corrigiendo las desviaciones
detectadas para ROS 2 Jazzy**:

1. **Comandos APT depurados.** El informe agrupa todos los paquetes en una
   única invocación `apt install` que mezcla nombres reales con otros
   inexistentes o redundantes. Aquí se ha segmentado por funcionalidad
   (control, MoveIt, Gazebo, URDF) usando **solo binarios verificados** del
   repositorio `packages.ros.org/ros2/ubuntu` para `jazzy`.
2. **`ros-jazzy-tf-transformations`** no se incluye porque ningún nodo del
   paquete lo importa.
3. **Plugin de control:** se confirma `gz_ros2_control/GazeboSimSystem` y
   `gz_ros2_control-system` como las cadenas correctas para Harmonic; los
   plugins de Gazebo Classic citados en otras guías son inválidos.
4. **`moveit_py` es la API oficial** en Jazzy; el ejemplo del nodo usa
   `get_planning_component(...)` (no el obsoleto `get_planning_group`).
5. **Frecuencia del controlador:** se mantiene `update_rate: 250` Hz como
   indica el informe; valores inferiores generan saltos visibles en Gazebo.

---

## Licencia y mantenimiento

- **Mantenedor:** `mpereira <mpereira@todo.todo>`
- **Licencia:** *TODO: License declaration* (actualizar en `package.xml`).
- **Compatibilidad probada:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic ·
  MoveIt 2.

