# Proyecto Ajedrez Dual ABB IRB120 — Documentación Completa

**Sistema de ajedrez robótico con dos manipuladores ABB IRB120 manejados por voz, percepción y MoveIt 2 sobre ROS 2 Jazzy y Gazebo Harmonic.**

Este documento es la guía única de referencia: desde una máquina recién instalada hasta una partida completa controlada por voz. Está organizado para que puedas seguirlo en orden o saltar a la sección que necesites.

---

## Índice

1. [Resumen del proyecto](#1-resumen-del-proyecto)
2. [Arquitectura y flujo de datos](#2-arquitectura-y-flujo-de-datos)
3. [Requisitos del sistema](#3-requisitos-del-sistema)
4. [Instalación desde cero (paso a paso)](#4-instalación-desde-cero-paso-a-paso)
   1. [Ubuntu 24.04 / WSL2](#41-ubuntu-2404--wsl2)
   2. [ROS 2 Jazzy](#42-ros-2-jazzy)
   3. [Gazebo Harmonic](#43-gazebo-harmonic)
   4. [MoveIt 2](#44-moveit-2)
   5. [Dependencias Python (IA, voz, visión)](#45-dependencias-python-ia-voz-visión)
   6. [Tokens de HuggingFace](#46-tokens-de-huggingface-opcional)
5. [Estructura del workspace](#5-estructura-del-workspace)
6. [Compilación del workspace](#6-compilación-del-workspace)
7. [Configuración y parámetros clave](#7-configuración-y-parámetros-clave)
8. [Lanzamiento del proyecto (3 modos)](#8-lanzamiento-del-proyecto-3-modos)
9. [Cómo dar órdenes por voz](#9-cómo-dar-órdenes-por-voz)
10. [Integración de modelos de piezas](#10-integración-de-modelos-de-piezas)
11. [Entrenamiento e integración de YOLO](#11-entrenamiento-e-integración-de-yolo)
12. [Interfaces ROS 2 (topics, services, actions)](#12-interfaces-ros-2-topics-services-actions)
13. [Resolución de problemas frecuentes](#13-resolución-de-problemas-frecuentes)
14. [Desarrollo y extensión del sistema](#14-desarrollo-y-extensión-del-sistema)
15. [Convenciones y principios de diseño](#15-convenciones-y-principios-de-diseño)
16. [Licencias y créditos](#16-licencias-y-créditos)

---

## 1. Resumen del proyecto

Dos brazos manipuladores **ABB IRB120** colocados frente a frente sobre una mesa juegan al ajedrez en simulación **Gazebo Harmonic**. Las partes son:

- Un **tablero** y **32 piezas** con una pequeña cabeza ferromagnética.
- Una **cámara cenital** que observa el tablero y publica imagen en ROS 2.
- Una **pipeline de percepción** que estima el estado del tablero a partir de la imagen (YOLOv8 + homografía).
- Una **pipeline de voz** que captura el micrófono, transcribe con **Whisper** de HuggingFace y convierte la frase a un movimiento estructurado con **LangChain** sobre un LLM (chat HuggingFace).
- Un **gestor de partida** que valida cada movimiento contra las reglas de ajedrez (`python-chess`).
- Un **ejecutor de movimientos** basado en **MoveIt 2** que planifica y ejecuta secuencias *pick & place* con un gripper magnético sobre el flange de cada robot.

El proyecto se asienta sobre el paquete `irb120_jazzy_sim` de la asignatura (TAREA 3), del que **no se modifica nada**: solo se reutilizan sus mallas y se compone una escena dual nueva en paquetes paralelos.

---

## 2. Arquitectura y flujo de datos

### 2.1 Mapa de paquetes

| Paquete | Tipo | Responsabilidad |
|---|---|---|
| `irb120_jazzy_sim` | `ament_python` | (TAREA 3, **intacto**) URDF del IRB120, MoveIt de un solo brazo. |
| `chess_msgs` | `ament_cmake` | Mensajes, servicios y acciones del proyecto. |
| `chess_description` | `ament_python` | URDFs de tablero, piezas, gripper magnético, **escena dual**. |
| `chess_moveit_config` | `ament_python` | MoveIt 2 config para los dos brazos. |
| `chess_gazebo` | `ament_python` | Mundo Gazebo, spawning de robots y piezas. |
| `chess_perception` | `ament_python` | Cámara cenital → estado del tablero (YOLOv8). |
| `chess_brain` | `ament_python` | Motor de ajedrez (`python-chess`) + gestor de partida. |
| `chess_voice` | `ament_python` | Micro → Whisper → LangChain → `ChessMove`. |
| `chess_motion` | `ament_python` | MoveItPy pick & place + attach/detach Gazebo. |
| `chess_bringup` | `ament_python` | Launch global y configuraciones. |

### 2.2 Flujo de datos en una jugada

```
  ┌──────────────────────────────────────────────────────────────────────┐
  │                          Gazebo Harmonic                            │
  │   dual_irb120_chess (URDF) + chess_world.sdf + 32 piezas spawned    │
  │     │                                                ▲              │
  │     │ /overhead_camera/image (sensor_msgs/Image)     │ joint cmds   │
  │     ▼                                                │              │
  │  chess_perception ──► /chess/perceived_state         │              │
  │                                                      │              │
  │   ┌─ usuario habla ─► mic ─► audio_capture ─► /chess/voice/audio    │
  │   │                                            │                    │
  │   │                                            ▼                    │
  │   │                                       whisper_asr                │
  │   │                                            │                    │
  │   │                              /chess/voice/utterance              │
  │   │                                            │                    │
  │   │                                            ▼                    │
  │   │                                     voice_parser                  │
  │   │                                            │                    │
  │   │                              /chess/voice/parse (srv)             │
  │   │                                            │                    │
  │   ▼                                            ▼                    │
  │  GameManager (python-chess) ◄─────────────── parsed ChessMove        │
  │      │                                                              │
  │      │  ExecuteChessMove (action) /chess/execute_move                │
  │      ▼                                                              │
  │   MoveExecutor (MoveItPy)                                            │
  │      │                                                              │
  │      ├─► JointTrajectory → controllers ────────────────────────────► │
  │      └─► gz CLI attach/detach pieces                                 │
  └──────────────────────────────────────────────────────────────────────┘
```

### 2.3 Estrategia de agarre

- **Mecánicamente**: un cilindro corto magnético solidario al `tool0` de cada IRB120, terminado en un frame `gripper_tip` que MoveIt usa como TCP. Cada pieza lleva un pequeño disco ferromagnético en su parte superior.
- **En simulación**: el mundo carga el plugin `gz-sim-detachable-joint-system`. `chess_motion` invoca el servicio `/world/chess_world/detachable_joint/{attach,detach}` para crear o eliminar dinámicamente una unión fija entre el `gripper_tip` y la base de la pieza.
- **Por qué no jaw gripper**: las piezas son pequeñas, el foco del proyecto es voz + percepción + planificación, y el gripper magnético reduce drásticamente la complejidad de configurar MoveIt y depurar colisiones.

---

## 3. Instalación desde cero (paso a paso)

### 3.1 Ubuntu 24.04 / WSL2

Si estás en Windows:

```powershell
wsl --install -d Ubuntu-24.04
```

Reinicia, abre Ubuntu, crea tu usuario, y dentro de la terminal:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y curl gnupg lsb-release software-properties-common \
                    build-essential cmake git python3-pip python3-venv \
                    python3-colcon-common-extensions python3-vcstool \
                    python3-rosdep python3-argcomplete locales
sudo locale-gen en_US.UTF-8 es_ES.UTF-8
```

### 3.2 ROS 2 Jazzy

```bash
# Repositorio ROS 2 oficial
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Instalación
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Sourceado automático en cada terminal
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# rosdep
sudo rosdep init || true
rosdep update
```

Verifica:

```bash
echo $ROS_DISTRO         # → jazzy
ros2 doctor              # → debería pasar todos los checks
```

### 3.3 Gazebo Harmonic

Jazzy y Harmonic emparejan oficialmente:

```bash
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-controller-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro
```

Comprueba:

```bash
gz sim --version       # → Harmonic
```

### 3.4 MoveIt 2

```bash
sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-rviz2
```

### 3.5 Dependencias Python (IA, voz, visión)

Se recomienda **fuertemente** un entorno virtual aparte para no contaminar el sistema:

```bash
python3 -m venv ~/.venvs/chess
source ~/.venvs/chess/bin/activate

pip install --upgrade pip wheel

# Ajedrez + percepción
pip install python-chess opencv-python ultralytics

# Voz (Whisper + audio + LangChain + LLM HF)
pip install numpy sounddevice transformers torch accelerate \
            langchain langchain-core langchain-huggingface huggingface_hub

# Utilidades varias
pip install tf-transformations PyYAML
```

> El paquete `tf-transformations` ya existe en apt como `ros-jazzy-tf-transformations`. Si prefieres apt: `sudo apt install -y ros-jazzy-tf-transformations` y no lo pongas en el venv.

Para que ROS 2 use ese venv, abre cada terminal así:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.venvs/chess/bin/activate
```

### 3.6 Tokens de Hugging Face (opcional)

Si quieres que LangChain consulte un LLM en la nube (recomendado: Llama-3-8B-Instruct):

1. Crea cuenta en https://huggingface.co.
2. Crea un **Access Token** con permiso `Read`.
3. Acepta los términos del modelo (en su página) si los hay.
4. En tu shell:

```bash
export HUGGINGFACEHUB_API_TOKEN=hf_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
echo 'export HUGGINGFACEHUB_API_TOKEN=hf_xxx' >> ~/.bashrc
```

> Si no quieres usar LLM, el sistema dispone de un **parser por regex** como fallback: `use_llm:=false` en el launch. Cubre comandos comunes (de–a, captura, enroque, casilla con figura).

### Comandos para verificar las 3 secciones:

#### 3.3 — Gazebo Harmonic y ros2_control:


gz sim --version && \
dpkg -l ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller 2>/dev/null | awk '/^ii/{print $2" OK"} /^[^i]/{print $2" FALTA"}'

#### 3.4 — MoveIt 2:


dpkg -l ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-configs-utils ros-jazzy-rviz2 2>/dev/null | awk '/^ii/{print $2" OK"} /^[^i]/{print $2" FALTA"}'

#### 3.5 — Python (venv chess):


source ~/.venvs/chess/bin/activate 2>/dev/null || echo "VENV NO EXISTE: ~/.venvs/chess"
python3 -c "
import importlib, sys
pkgs = ['chess','cv2','ultralytics','sounddevice','torch','transformers','accelerate','langchain','huggingface_hub','yaml']
for p in pkgs:
    try: importlib.import_module(p); print(p,'OK')
    except: print(p,'FALTA')
"

**Copia y pega cada bloque por separado. Cualquier línea que diga FALTA te indica exactamente qué no está instalado (menos en el 4.5, donde el sounddevice puede darse como falta)**

**PARA ASEGURARSE Y MATAR TODOS LOS PROCESOS: sudo pkill -f ros2**

---

## 4. Estructura del workspace

```
~/Robotica_inteligente/ros2_ws/
├── README.md                       (informe y notas previas)
├── README_PROYECTO.md              (este documento)
├── src/
│   ├── irb120_jazzy_sim/           (TAREA 3 — INTACTO)
│   ├── chess_msgs/                 (interfaces)
│   ├── chess_description/          (URDFs y meshes auxiliares)
│   ├── chess_moveit_config/        (MoveIt 2 dual-brazo)
│   ├── chess_gazebo/               (mundo y spawning)
│   ├── chess_perception/           (cámara → BoardState)
│   ├── chess_brain/                (motor y orquestador)
│   ├── chess_voice/                (mic → Whisper → LangChain → parser)
│   ├── chess_motion/               (MoveItPy pick & place)
│   └── chess_bringup/              (launch global)
├── build/                          (generado)
├── install/                        (generado)
└── log/                            (generado)
```

Documentación interna relevante:

- `src/chess_bringup/README_PROJECT.md` — guía corta de referencia.
- `src/chess_perception/models/README_TRAINING.md` — cómo entrenar YOLO.
- `src/chess_gazebo/config/board_layout.yaml` — geometría del tablero.

---

## 5. Compilación del workspace

### 5.1 Resolver dependencias del manifiesto

Desde la raíz del workspace y con ROS sourcado:

```bash
cd ~/Robotica_inteligente/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y \
               --skip-keys "moveit_py langchain langchain-huggingface ultralytics sounddevice"
```

> Las claves saltadas son librerías Python que se instalan vía `pip`, no vía apt.

### 5.2 Compilar

```bash
colcon build --symlink-install
```

La primera vez tarda ~3–5 minutos (la generación de interfaces de `chess_msgs` es la parte más lenta).

### 5.3 Sourcear el overlay

```bash
source install/setup.bash
```

> Recomendado: añade a `~/.bashrc`:
> ```bash
> source /opt/ros/jazzy/setup.bash
> [ -f ~/Robotica_inteligente/ros2_ws/install/setup.bash ] \
>   && source ~/Robotica_inteligente/ros2_ws/install/setup.bash
> ```

### 5.4 Recompilar tras editar interfaces

```bash
colcon build --packages-up-to chess_msgs --symlink-install
# o todo:
colcon build --symlink-install
```

---

## 6. Configuración y parámetros clave

Todos los parámetros relevantes se leen de YAML o de los argumentos del launch.

### 6.1 Geometría del tablero (`chess_gazebo/config/board_layout.yaml`)

```yaml
board_size: 0.40           # m, lado útil del tablero
square_size: 0.05          # = board_size / 8
board_z: 0.02              # altura superior del tablero (encima de la mesa)
board_centre: [0.0, 0.0]   # centrado en el origen del mundo
approach_clearance: 0.08   # m de aproximación encima de la pieza
grasp_clearance: 0.005     # m de holgura sobre el imán de la pieza
initial_fen: "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
pieces:
  pawn:   {height: 0.045, radius: 0.012, mass: 0.020, mesh_uri: ""}
  rook:   {height: 0.055, radius: 0.014, mass: 0.030, mesh_uri: ""}
  knight: {height: 0.060, radius: 0.014, mass: 0.030, mesh_uri: ""}
  bishop: {height: 0.065, radius: 0.013, mass: 0.030, mesh_uri: ""}
  queen:  {height: 0.080, radius: 0.015, mass: 0.040, mesh_uri: ""}
  king:   {height: 0.085, radius: 0.015, mass: 0.040, mesh_uri: ""}
```

Cambia `mesh_uri` para cargar tus STLs (ver §10).

### 6.2 Argumentos del launch global

| Argumento | Default | Descripción |
|---|---|---|
| `use_llm` | `true` | Usa LangChain + ChatHuggingFace; si `false`, fallback regex |
| `llm_model_id` | `meta-llama/Meta-Llama-3-8B-Instruct` | Repo HF para el LLM |
| `yolo_weights` | `""` | Ruta al `.pt`. Si vacío, perception publica tablero vacío |
| `whisper_model` | `openai/whisper-small` | Modelo HF para ASR |
| `whisper_device` | `cpu` | `cpu` o `cuda` |
| `enable_voice` | `true` | (reservado para futuros launch reducidos) |
| `enable_vision` | `true` | (reservado para futuros launch reducidos) |

### 6.3 Posición de los robots

En `chess_description/urdf/dual_irb120_chess.urdf.xacro` el argumento `robot_spacing` (default `0.45`) coloca cada base a ±0.45 m del centro del tablero en el eje X. El brazo *black* gira 180° (yaw=π).

---

## 7. Lanzamiento del proyecto (3 modos)

> Asegúrate siempre de tener ROS sourcado **y** el venv de Python activado.

### 7.1 Modo simulación + MoveIt (iteración rápida, sin voz ni visión)

```bash
ros2 launch chess_bringup chess_sim_only.launch.py
```

Arranca:

- Gazebo con `chess_world.sdf`
- Robot dual + controladores (`joint_state_broadcaster`, `white_arm_controller`, `black_arm_controller`)
- Spawner de piezas (32 piezas, posición inicial estándar)
- MoveIt 2 con RViz mostrando los dos grupos `white_arm`/`black_arm`

Útil para depurar planificación, colisiones, mallas, etc.

### 7.2 Modo bypass de micrófono (testing pipeline completa por texto)

Terminal 1:

```bash
ros2 launch chess_bringup chess_full.launch.py \
    use_llm:=false \
    yolo_weights:=""
```

> Sin LLM (más rápido en el desarrollo); el regex fallback entiende: "peón de e2 a e4", "caballo a f3", "torre captura en d5", "enroque corto", "enroque largo".

Terminal 2 (publica una orden):

```bash
ros2 topic pub -1 /chess/voice/utterance std_msgs/msg/String \
    "{data: 'peón de e2 a e4'}"
```

Observa cómo el robot blanco recoge el peón y lo mueve.

### 7.3 Modo completo (voz + LLM + percepción)

Todos los parámetros se gestionan por defecto desde el archivo de configuración central en `src/chess_voice/config/params.yaml`.

Para usar el LLM de Hugging Face de forma transparente sin declarar variables en el terminal, edite `src/chess_voice/config/params.yaml` e introduzca su clave de API:

```yaml
chess_voice_parser:
  ros__parameters:
    huggingface_api_token: "tu_token_aqui"
```

> ⚠️ **Seguridad:** NO commitees `params.yaml` con un token real. El método recomendado y por defecto es dejar `huggingface_api_token: ""` y autenticarte una vez con `huggingface-cli login` (el token se guarda en `~/.cache/huggingface/token`, fuera de Git). Ver *Ejecución demo 7.3*.

A continuación, simplemente ejecute:

```bash
ros2 launch chess_bringup chess_full.launch.py
```

Si aún desea sobrescribir parámetros dinámicamente desde la línea de comandos (por ejemplo, para cambiar de modelo o dispositivo), puede hacerlo del siguiente modo:

```bash
ros2 launch chess_bringup chess_full.launch.py \
    use_llm:=true \
    llm_model_id:=meta-llama/Meta-Llama-3-8B-Instruct \
    whisper_model:=openai/whisper-small \
    whisper_device:=cpu
```

Habla cerca del micrófono. La pipeline:

1. `audio_capture` segmenta tu intervención cuando dejas de hablar (~700 ms de silencio).
2. `whisper_asr` transcribe a texto.
3. `voice_parser` (LLM) convierte el texto a UCI.
4. `game_manager` valida la jugada con `python-chess`.
5. `move_executor` planifica y ejecuta el pick & place.
6. `board_state_estimator` publica `/chess/perceived_state`. El `game_manager` lo usa como **validador advisory oportunista**: nunca bloquea, nunca trata una casilla no detectada como vacía, y solo avisa si una detección de alta confianza contradice una casilla ocupada del motor (ver *Ejecución demo 7.3* más abajo).

---

## Ejecución demo 7.3 (reproducible, sin Docker)

Receta mínima para que, tras `git pull`, cualquiera ponga en marcha la demo de
voz + LLM. La visión YOLO queda como **advisory experimental** (no es autoridad
del tablero), por lo que la demo principal va **sin visión**.

### 1. Preparar el entorno ROS

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

> El `-DPython3_EXECUTABLE=/usr/bin/python3` es obligatorio: si CMake elige otro
> intérprete, `chess_msgs` falla en `rosidl`.

### 2. Preparar la IA (entorno `/opt/ai-venv`, numpy<2)

```bash
bash scripts/setup_ai_venv.sh
/opt/ai-venv/bin/huggingface-cli login   # pega un token Read; NO se versiona
```

`scripts/setup_ai_venv.sh` crea `/opt/ai-venv`, instala `requirements-ai.txt` y
**re-fija `numpy<2`** al final (ROS/cv_bridge se rompen con numpy 2). No toca el
Python del sistema ni guarda ningún token.

### 3. Comprobar el micrófono (WSL2)

```bash
pactl info
pactl list sources short

# El check de Python necesita el puente IA (sounddevice vive en /opt/ai-venv):
export PYTHONPATH=/opt/ai-venv/lib/python3.12/site-packages:$PYTHONPATH
python3 -c "import sounddevice as sd; print(sd.query_devices()); print(sd.default.device)"
```

> En WSL2 el audio entra por WSLg/PulseAudio. Si `sounddevice` no ve ninguna
> entrada, revisa que tu host (Windows) expone un micrófono y que
> `PULSE_SERVER=unix:/mnt/wslg/PulseServer` (lo exporta el script de demo).

### 4. Lanzar la demo

```bash
bash scripts/run_demo_73.sh
```

Equivale a, con el puente IA y el audio ya exportados:

```bash
ros2 launch chess_bringup chess_full.launch.py enable_voice:=true enable_vision:=false
```

### 5. Dar una orden de voz

> "mueve el alfil blanco de d3 a c4"

Más comandos de voz verificados (con sus UCI/SAN y la secuencia recomendada para
la demo) en [`docs/JUGADAS_DEMO_VOZ.md`](docs/JUGADAS_DEMO_VOZ.md).

### 6. Resultado esperado

1. `whisper_asr` transcribe el audio.
2. `voice_parser` (LLM) devuelve `d3c4`.
3. `game_manager` despacha `Bc4+`.
4. `move_executor` planifica y el brazo ejecuta el pick & place.
5. La pieza sigue al gripper durante el transporte.
6. `Move applied. Turn: black`.

### Visión advisory (opcional, experimental)

```bash
ros2 launch chess_bringup chess_full.launch.py enable_voice:=false enable_vision:=true
```

- La percepción YOLO es **solo un validador advisory/oportunista**: nunca bloquea
  jugadas y **no debe usarse como autoridad del tablero**.
- `best.pt` es el modelo por defecto (ruta absoluta, `conf_threshold:0.35`). En la
  escena actual de Gazebo detecta de forma parcial (1–2 piezas) por la brecha de
  dominio real→simulación; por eso es advisory.
- El motor de ajedrez (`python-chess`) sigue siendo la **fuente de verdad**; la
  visión solo avisa ante una contradicción fuerte con suficiente confianza.
- Para una visión fiable haría falta **fine-tuning con renders del propio
  simulador** (fuera del alcance de esta entrega).
- `best_yolox.pt` **no se versiona** y no compensa en la simulación actual.
- El **token HF nunca se commitea** (usa `huggingface-cli login`).

---

## 8. Cómo dar órdenes por voz

El parser entiende español e inglés mezclados, y dispone de ejemplos de pocos disparos. Cualquiera de estos funciona:

| Frase | Resultado (UCI) |
|---|---|
| *"peón de e2 a e4"* | `e2e4` |
| *"caballo a f3"* | `g1f3` |
| *"alfil c4"* | `f1c4` |
| *"captura en d5 con la torre"* | la única captura legal de torre a d5 |
| *"enroque corto"* | `e1g1` o `e8g8` según turno |
| *"enroque largo"* | `e1c1` o `e8c8` |
| *"corono peón en e8 a dama"* | `e7e8q` |

Reglas internas:

- El parser solo emite UCI que esté en la lista de movimientos legales del FEN actual.
- Si la frase no es interpretable, `success=false` y `game_manager` lo registra sin mover el robot.
- Si la jugada es ilegal según `python-chess`, el robot no se mueve.

### 8.1 Test desde terminal (sin micro)

```bash
ros2 topic pub -1 /chess/voice/utterance std_msgs/msg/String "{data: 'caballo a f3'}"
```

---

## 9. Integración de modelos de piezas

El xacro `chess_description/urdf/chess_piece.urdf.xacro` está preparado para dos modos: **primitiva geométrica** (default, cilindro) o **STL real**. Hay tres caminos según tu situación.

### 9.1 Camino A — STL abierto de Internet (rápido)

Repositorios recomendados (todos con licencia libre o CC):

- **`marcusmaier/chess-pieces`** en GitHub — pack `.stl` completo.
- **`StoneyKey/chess-pieces`** — alternativo, formas redondeadas.
- **Wikimedia Commons → "3D chess pieces"** — varios sets CC-BY-SA.
- **Thingiverse** (revisar licencia de cada modelo).

Procedimiento:

```bash
cd ~/Robotica_inteligente/ros2_ws/src/chess_description/meshes/pieces
# Coloca aquí: pawn.stl, rook.stl, knight.stl, bishop.stl, queen.stl, king.stl
```

Edita `src/chess_gazebo/config/board_layout.yaml`:

```yaml
pieces:
  pawn:
    height: 0.045
    radius: 0.012
    mass: 0.020
    mesh_uri: "package://chess_description/meshes/pieces/pawn.stl"
  rook:
    ...
```

Si tu STL viene en **milímetros**, el xacro ya aplica `scale="0.001 0.001 0.001"`. Si viene en **metros**, edita `chess_piece.urdf.xacro` y quita el `scale` (o ponlo `1 1 1`).

Recompila y relanza:

```bash
colcon build --packages-up-to chess_description chess_gazebo --symlink-install
source install/setup.bash
ros2 launch chess_bringup chess_sim_only.launch.py
```

### 9.2 Camino B — STL propios (impresión 3D)

Mismo procedimiento que A, pero ajusta `height` al alto real medido de tus prints. La altura es importante porque el planificador de poses calcula `grasp_z = board_z + height + grasp_clearance`.

### 9.3 Camino C — Pack que incluya dataset YOLO

El dataset **Roboflow "Chess Pieces Detection"** suele acompañarse de su set de STL imprimibles. Esto permite usar las mismas piezas en simulación y en el modelo de visión, garantizando un dominio coherente.

URL: https://universe.roboflow.com/joseph-nelson/chess-pieces-new

- Descarga el dataset en formato **YOLOv8**.
- Renombra las clases (si vienen como `P`, `K`, etc.) a:
  ```
  white_pawn, white_knight, white_bishop, white_rook, white_queen, white_king,
  black_pawn, black_knight, black_bishop, black_rook, black_queen, black_king
  ```
- Si el repo no provee STLs, los puedes generar simples a partir de los renders del dataset.

---

## 10. Entrenamiento e integración de YOLO

### 10.1 Entrenar (Ultralytics YOLOv8)

```bash
source ~/.venvs/chess/bin/activate
mkdir -p ~/chess_yolo && cd ~/chess_yolo

# Suponiendo que has descargado el dataset y tienes un chess.yaml válido
yolo detect train \
    data=chess.yaml \
    model=yolov8n.pt \
    epochs=80 \
    imgsz=640 \
    batch=16 \
    device=0          # 'cpu' si no tienes GPU
```

El mejor checkpoint queda en `runs/detect/train/weights/best.pt`.

### 10.2 Instalación del modelo

Copia el `best.pt` a un directorio estable, por ejemplo:

```bash
cp runs/detect/train/weights/best.pt \
   ~/Robotica_inteligente/ros2_ws/src/chess_perception/models/best.pt
```

Y lánzalo con:

```bash
ros2 launch chess_bringup chess_full.launch.py \
    yolo_weights:=$HOME/Robotica_inteligente/ros2_ws/src/chess_perception/models/best.pt
```

### 10.3 Sin modelo entrenado todavía

El sistema arranca igualmente: `board_state_estimator` carga `NullDetector` y publica `/chess/perceived_state` con tablero vacío. El motor sigue siendo la fuente de verdad, así que **no necesitas YOLO para jugar**. Lo necesitas para validar el estado contra la realidad y para futuras extensiones (modo "robot vs humano" donde el humano mueve a mano y el sistema percibe).

---

## 11. Interfaces ROS 2 (topics, services, actions)

### 11.1 Topics

| Topic | Tipo | Quién publica | Quién suscribe |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (bridge) | Todos los nodos `use_sim_time=true` |
| `/overhead_camera/image` | `sensor_msgs/Image` | Gazebo (bridge) | `chess_perception` |
| `/chess/voice/audio` | `std_msgs/Float32MultiArray` | `audio_capture` | `whisper_asr` |
| `/chess/voice/utterance` | `std_msgs/String` | `whisper_asr` | `game_manager` |
| `/chess/board_state` | `chess_msgs/BoardState` | `game_manager` (autoridad) | UI, debug |
| `/chess/perceived_state` | `chess_msgs/BoardState` | `chess_perception` | `game_manager` (cross-check) |
| `/joint_states` | `sensor_msgs/JointState` | controllers | MoveIt, RViz |
| `/<color>_arm_controller/follow_joint_trajectory` | action | MoveIt | controllers |

### 11.2 Servicios

| Servicio | Tipo | Servidor |
|---|---|---|
| `/chess/voice/parse` | `chess_msgs/ParseVoiceCommand` | `voice_parser` |
| `/world/chess_world/create` | `gz.msgs.EntityFactory` | Gazebo |
| `/world/chess_world/remove` | `gz.msgs.Entity` | Gazebo |
| `/world/chess_world/detachable_joint/attach` | `gz.msgs.DetachableJoint` | Gazebo plugin |
| `/world/chess_world/detachable_joint/detach` | `gz.msgs.DetachableJoint` | Gazebo plugin |

### 11.3 Acciones

| Acción | Tipo | Servidor |
|---|---|---|
| `/chess/execute_move` | `chess_msgs/ExecuteChessMove` | `chess_motion` |

Cliente típico: `chess_brain.game_manager`.

### 11.4 Inspección rápida

```bash
ros2 topic list
ros2 topic echo /chess/board_state
ros2 service list | grep chess
ros2 action list | grep chess
ros2 param dump /chess_game_manager
```

---

## 12. Resolución de problemas frecuentes

### 12.1 `CMake Error ... ament_cmake ... not found`

ROS 2 no está sourceado. Ejecuta:

```bash
source /opt/ros/jazzy/setup.bash
```

y vuelve a `colcon build`.

### 12.2 `gz: command not found`

Falta Gazebo Harmonic. Repite §4.3.

### 12.3 Gazebo arranca pero no aparecen los robots

- Revisa que el `xacro` del `dual_irb120_chess` se evalúe sin errores:
  ```bash
  ros2 run xacro xacro $(ros2 pkg prefix chess_description)/share/chess_description/urdf/dual_irb120_chess.urdf.xacro use_gazebo:=true | head
  ```
- Verifica que `/robot_description` se está publicando:
  ```bash
  ros2 topic echo /robot_description --once | head -5
  ```

### 12.4 Los controladores no spawnean

```bash
ros2 control list_controllers
```

Si no salen, comprueba `controller_manager`:

```bash
ros2 node list | grep controller_manager
```

Y mira en los logs de Gazebo si `gz_ros2_control-system` se cargó. Posibles causas: falta `ros-jazzy-gz-ros2-control`, o un xacro lanzó dos plugins (la macro `dual_irb120_ros2control` solo emite uno; cualquier duplicado provoca conflicto).

### 12.5 Las piezas no aparecen

`spawn_chess_pieces` se ejecuta 6 s después del `gz_sim`. Revisa:

```bash
ros2 run chess_gazebo spawn_chess_pieces --ros-args \
    -p world:=chess_world \
    -p initial_delay:=0.0
```

Si falla, prueba el servicio manualmente:

```bash
gz service -s /world/chess_world/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req 'sdf_filename: "/tmp/test.urdf", name: "test", pose: {position: {z: 0.5}}'
```

### 12.6 Whisper se cuelga al arrancar

La primera vez descarga el modelo (~500 MB para `small`). Espera 1–2 minutos. Para acelerar:

```bash
huggingface-cli download openai/whisper-small
```

### 12.7 LangChain devuelve error 401/403

Falta o caducó tu `HUGGINGFACEHUB_API_TOKEN`, o no aceptaste los términos del modelo. Cambia a un modelo abierto, p.ej. `HuggingFaceH4/zephyr-7b-beta`, o usa `use_llm:=false`.

### 12.8 MoveIt no encuentra IK / planning fails

- Verifica que `white_gripper_tip` / `black_gripper_tip` existen como links:
  ```bash
  ros2 run tf2_tools view_frames
  ```
- Aumenta `kinematics_solver_timeout` en `chess_moveit_config/config/kinematics.yaml` a `0.2`.
- Posiblemente la pose objetivo está fuera del workspace: confirma que el tablero está dentro de un radio ~0.45 m de la base.

### 12.9 Errores de audio (`PortAudio not found`)

```bash
sudo apt install -y portaudio19-dev libsndfile1
pip install --force-reinstall sounddevice
```

En WSL2 necesitas WSLg + Windows expone el micro:

```powershell
# en Windows PowerShell
Get-PnpDevice -Class AudioEndpoint -Status OK
```

### 12.10 Reinstalación limpia

```bash
cd ~/Robotica_inteligente/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 13. Desarrollo y extensión del sistema

### 13.1 Añadir un comando de voz nuevo

- Si es una frase nueva con la misma intención (mover una pieza), no necesitas tocar nada: el LLM lo entenderá.
- Si introduces una **nueva intención** (p.ej. "deshaz la última jugada"), añade:
  - una rama en `chess_voice/parsers.py` (regex fallback),
  - una acción en `chess_msgs/action/`, y
  - su manejador en `chess_brain/game_manager.py`.

### 13.2 Cambiar el motor de IA del oponente

Hoy el motor solo aplica reglas y valida. Para que la máquina mueva sola un bando:

1. Instala Stockfish: `sudo apt install -y stockfish`.
2. Crea `chess_brain/stockfish_adapter.py` envolviendo `python-chess.engine.SimpleEngine.popen_uci('stockfish')`.
3. En `game_manager.py`, cuando sea el turno del color "máquina", llama al adaptador y reusa el mismo despacho a `chess_motion`.

### 13.3 Reemplazar Whisper por un modelo local más rápido

`whisper_asr_node.py` carga vía `transformers.pipeline`. Cambia `model:=openai/whisper-tiny` (más rápido) o usa **faster-whisper**:

```python
# en lugar del pipeline:
from faster_whisper import WhisperModel
model = WhisperModel("small", device="cpu", compute_type="int8")
segments, _ = model.transcribe(audio, language="es")
text = "".join(s.text for s in segments)
```

### 13.4 Calibración en hardware real

`chess_perception/homography.py` incluye `IntrinsicHomography` (cámara cenital pura) y deja preparado `MarkerHomography`. En real:

- Coloca 4 ArUcos en las esquinas del tablero.
- Detecta sus centros con `cv2.aruco`.
- Llama a `cv2.findHomography(image_pts, world_pts)` cada N segundos o on-demand.
- Sustituye `IntrinsicHomography` por `MarkerHomography` en el nodo.

### 13.5 Añadir un reloj de ajedrez

Crear un nodo `chess_clock` que mantenga dos contadores y publique en `/chess/clock_state`. Suscribirse a `/chess/board_state` para detectar cambio de turno y conmutar.

---

## 14. Convenciones y principios de diseño

### 14.1 SRP — Una responsabilidad por nodo

- `audio_capture` solo captura y segmenta audio.
- `whisper_asr` solo transcribe.
- `voice_parser` solo convierte texto → UCI legal.
- `game_manager` solo valida + orquesta.
- `move_executor` solo planifica y ejecuta MoveIt.
- `board_state_estimator` solo observa.

### 14.2 OCP / DIP — Programar contra interfaces

`PieceDetector`, `MoveParser`, `ChessEngine` son `Protocol`s. Cualquier backend nuevo se enchufa sin tocar los nodos consumidores.

### 14.3 LSP — Sustituibilidad real

- `RegexFallbackParser` ↔ `LangChainLLMParser`.
- `NullDetector` ↔ `YoloPieceDetector`.
- `PythonChessEngine` ↔ otra implementación de `ChessEngine`.

### 14.4 ISP — Servicios estrechos

`ParseVoiceCommand`, `SetGripper`, `AttachPiece` están separados en lugar de un solo "ChessRobotCommand" con muchos campos opcionales.

### 14.5 Convenciones de naming

- Prefijo `white_` / `black_` en todos los joints/links del robot.
- Modelos Gazebo de piezas: `<color>_<piece>_<square>` (ej. `white_pawn_e2`).
- Frame del TCP: `<color>_gripper_tip`.
- Topics del proyecto: `/chess/...`.

### 14.6 Estilo Python

- Tipado estático con `typing` (`Protocol`, `Optional`, `List`).
- Dataclasses inmutables (`frozen=True`) para estructuras de datos.
- Sin comentarios redundantes; los pocos comentarios explican el *por qué*.

---

## 15. Licencias y créditos

| Componente | Origen | Licencia |
|---|---|---|
| ABB IRB120 URDF / meshes | IFRA-Cranfield, ROS-Industrial | Apache-2.0 |
| `irb120_jazzy_sim` (TAREA 3) | Asignatura de Robótica Inteligente | (según asignatura) |
| Paquetes `chess_*` | Este proyecto | Apache-2.0 |
| `python-chess` | Niklas Fiekas | GPL-3.0 |
| Whisper | OpenAI / HuggingFace | MIT |
| LangChain | LangChain Inc. | MIT |
| Ultralytics YOLOv8 | Ultralytics | AGPL-3.0 |

> ⚠️ **AGPL-3.0** de YOLOv8 implica que cualquier despliegue público debe liberar el código. Para uso académico está bien; para producto comercial, evalúa alternativas (YOLO-NAS, Detectron2 con MIT, etc.).

---

## Apéndice A. Comandos rápidos (cheatsheet)

```bash
# Sourcear ROS + venv
source /opt/ros/jazzy/setup.bash
source ~/.venvs/chess/bin/activate

# Build
cd ~/Robotica_inteligente/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Solo sim
ros2 launch chess_bringup chess_sim_only.launch.py

# Stack completo (LLM + Whisper + YOLO)
ros2 launch chess_bringup chess_full.launch.py \
    use_llm:=true yolo_weights:=$HOME/.../best.pt

# Stack sin LLM (rápido, regex)
ros2 launch chess_bringup chess_full.launch.py use_llm:=false

# Test pipeline sin micro
ros2 topic pub -1 /chess/voice/utterance std_msgs/msg/String \
    "{data: 'caballo a f3'}"

# Inspección
ros2 topic list
ros2 service list | grep chess
ros2 action list  | grep chess
ros2 topic echo /chess/board_state
ros2 control list_controllers

# Render manual de URDF
ros2 run xacro xacro \
    $(ros2 pkg prefix chess_description)/share/chess_description/urdf/dual_irb120_chess.urdf.xacro \
    use_gazebo:=true > /tmp/scene.urdf
```

## Apéndice B. Glosario

- **FEN**: notación Forsyth-Edwards, representación de una posición de ajedrez en un string.
- **UCI**: Universal Chess Interface, formato `e2e4` para movimientos.
- **SAN**: Standard Algebraic Notation, `Nf3`.
- **TCP**: Tool Center Point, el frame que MoveIt planifica.
- **MoveItPy**: API Python de MoveIt 2.
- **DetachableJoint**: plugin de Gazebo Harmonic para uniones fijas creadas en runtime.
- **ASR**: Automatic Speech Recognition.
- **LLM**: Large Language Model.
- **VAD**: Voice Activity Detection.
