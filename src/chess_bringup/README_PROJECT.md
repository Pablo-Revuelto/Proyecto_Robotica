# Dual ABB IRB120 — Voice-Controlled Robotic Chess

Two ABB IRB120 manipulators play chess against each other in Gazebo Harmonic,
driven by spoken commands. Built on top of the unmodified TAREA 3 package
(`irb120_jazzy_sim`).

> **The TAREA 3 package is untouched.** Everything new lives in the
> sibling `chess_*` packages under `src/`.

---

## 1. Package map

| Package | Type | Purpose |
|---|---|---|
| `chess_msgs` | `ament_cmake` | `Square`, `ChessMove`, `BoardState`, `DetectedPiece` messages; `ParseVoiceCommand`, `SetGripper`, `AttachPiece` services; `ExecuteChessMove` action. |
| `chess_description` | `ament_python` | URDF/xacro for board, parametric pieces, magnetic gripper, dual-arm scene; reuses TAREA 3 meshes via `$(find irb120_jazzy_sim)/meshes/...`. |
| `chess_moveit_config` | `ament_python` | SRDF with `white_arm`/`black_arm` planning groups, OMPL/KDL/limits configs, MoveIt launch. |
| `chess_gazebo` | `ament_python` | World SDF (table + overhead camera + detachable-joint plugin), spawning of robots + 32 pieces from a FEN. |
| `chess_perception` | `ament_python` | Overhead-camera board-state estimator. `YoloPieceDetector` (12-class YOLOv8) + analytical homography. |
| `chess_brain` | `ament_python` | Board geometry helpers, python-chess engine wrapper, `GameManager` orchestrator. |
| `chess_voice` | `ament_python` | Mic capture + VAD, HuggingFace Whisper ASR, LangChain LLM parser (+ regex fallback). |
| `chess_motion` | `ament_python` | MoveItPy pick-and-place action server `/chess/execute_move`, Gazebo attach/detach via `gz` CLI. |
| `chess_bringup` | `ament_python` | Top-level launch (`chess_full.launch.py`, `chess_sim_only.launch.py`, `test_utterance.launch.py`). |

---

## 2. Data-flow

```
            ┌──────────────────────────────────────────────────────────────┐
            │                       Gazebo Harmonic                       │
            │  dual_irb120_chess (URDF) + chess_world.sdf + 32 pieces     │
            │   ↘ /overhead_camera/image  ↗ controllers (joint_traj)      │
            └──────────────────────────────────────────────────────────────┘
                       │                            ↑
                       │ /overhead_camera/image      │ JointTrajectory
                       ▼                            │
              chess_perception                       │
                       │                            │
                       │ /chess/perceived_state      │
                       ▼                            │
                                                    │
   mic → audio_capture → whisper_asr → /chess/voice/utterance         │
                                              │                       │
                                              ▼                       │
                                      voice_parser (LangChain)         │
                                              │                       │
                                /chess/voice/parse (srv)               │
                                              │                       │
                                              ▼                       │
                                       chess_brain.GameManager ────────┘
                                              │
                                              │ ExecuteChessMove (action)
                                              ▼
                                       chess_motion.MoveExecutor
                                              │
                                              ▼  (gz CLI)
                                       attach/detach pieces
```

---

## 3. End-effector & grasping strategy

* **Mechanically:** a small cylindrical "magnetic shaft" rigidly fixed to
  `tool0`, ending in a `gripper_tip` frame that MoveIt treats as the TCP.
  Each chess piece has a small ferromagnetic disc on its head.
* **Simulation grasp:** the world includes `gz-sim-detachable-joint-system`.
  `chess_motion` calls `/world/chess_world/detachable_joint/{attach,detach}`
  to create or remove a temporary fixed joint between `<color>_gripper_tip`
  and the piece's base link. This is more robust than emulated grasp physics
  and is the recommended pattern for educational pick-and-place setups in
  Gazebo Harmonic.
* **Why not a parallel jaw?** Pieces are small and the project's focus is
  perception + voice; a magnetic gripper keeps the URDF / MoveIt config
  light, removes the need for a finger collision model, and is easier to
  port to a real magnetic EOAT on the actual IRB120.

---

## 4. Chess piece meshes — how to plug in real models

The chess piece xacro `chess_description/urdf/chess_piece.urdf.xacro` accepts
a `mesh_uri` argument. When empty (default), a parametric cylinder is used so
the simulation runs out of the box. **Three integration paths:**

### Option A — STL set from an open repository (recommended quick path)

1. Download an open-source 3D chess set. Two well-known options:
   - **Wikimedia "Standard chess pieces"** (CC-BY-SA) — search Wikimedia
     Commons for `Chess set 3D`.
   - **GitHub `marcusmaier/chess-pieces`** or
     **`StoneyKey/chess-pieces`** — simple .stl files of all 6 piece types.
   - **Thingiverse "STL files for chess pieces"** (verify each thing's
     licence).
2. Convert the OBJ/STL to a single STL per piece type, named:
   `pawn.stl, rook.stl, knight.stl, bishop.stl, queen.stl, king.stl`.
3. Drop them in `chess_description/meshes/pieces/`.
4. Edit `chess_gazebo/config/board_layout.yaml`, e.g.:
   ```yaml
   pieces:
     pawn: { height: 0.045, radius: 0.012, mass: 0.020,
             mesh_uri: "package://chess_description/meshes/pieces/pawn.stl" }
     ...
   ```
5. Rebuild (`colcon build --packages-up-to chess_description chess_gazebo`)
   and relaunch. The spawner re-renders the xacro per piece, picking up the
   mesh URI automatically.

> The xacro currently applies `scale="0.001 0.001 0.001"` to the mesh,
> appropriate for mm-unit STLs. Adjust if your STLs are in metres.

### Option B — 3D-print-grade STLs you already own

Same as Option A; just drop your own STLs in the same folder and edit
`board_layout.yaml`. Heights should match the real prints so the gripper
plans grasp Z correctly. A piece of total height H will be grasped at
`Z = board_z + H + grasp_clearance`.

### Option C — Mesh set that ships its own labelled YOLO dataset

Two-for-one: get a chess set whose creators also released a labelled
detection dataset. Recommended:

- **Roboflow Universe — "Chess Pieces Detection" (Joseph Nelson)** — 12-class
  YOLO dataset with a 3D-printed set whose STLs are linked in the dataset
  README. https://universe.roboflow.com/joseph-nelson/chess-pieces-new
- **roboflow-ai/chess-pieces-coco** — alternative COCO version.

Drop the YOLO weights at `chess_perception/models/best.pt` (after training,
see `chess_perception/models/README_TRAINING.md`) and the same STLs in
`chess_description/meshes/pieces/`. Both subsystems then use the same set.

---

## 5. Build

```bash
cd ~/Robotica_inteligente/ros2_ws

# ROS dependencies
rosdep install --from-paths src --ignore-src -y

# Python ML dependencies (one-time, in your environment of choice)
pip install python-chess transformers torch sounddevice numpy \
            langchain langchain-huggingface ultralytics opencv-python \
            tf-transformations

# Build everything
colcon build --symlink-install

source install/setup.bash
```

---

## 6. Run

### A) Just the simulation + MoveIt (for iteration)

```bash
ros2 launch chess_bringup chess_sim_only.launch.py
```

### B) Full stack (voice + perception + brain + motion)

All default parameters are managed from the central configuration file in `src/chess_voice/config/params.yaml`.

To use the HuggingFace LLM endpoint seamlessly without manual exports, edit `src/chess_voice/config/params.yaml` and set your API token:

```yaml
chess_voice_parser:
  ros__parameters:
    huggingface_api_token: "your_token_here"
```

Then, simply run:

```bash
ros2 launch chess_bringup chess_full.launch.py
```

If you still need to override parameters dynamically on the command line (e.g. to change the model or device):

```bash
ros2 launch chess_bringup chess_full.launch.py \
    use_llm:=true \
    llm_model_id:=meta-llama/Meta-Llama-3-8B-Instruct \
    whisper_model:=openai/whisper-small \
    whisper_device:=cpu
```

Speak: *"peón de e2 a e4"* / *"caballo a f3"* / *"enroque corto"* /
*"captura en d5 con la torre"*.

### C) Bypass the microphone (integration testing)

```bash
ros2 topic pub -1 /chess/voice/utterance std_msgs/msg/String \
    "{data: 'peon de e2 a e4'}"
```

---

## 7. ROS interfaces (cheatsheet)

| Direction | Name | Type |
|---|---|---|
| pub | `/chess/voice/audio` | `std_msgs/Float32MultiArray` |
| pub | `/chess/voice/utterance` | `std_msgs/String` |
| pub | `/chess/board_state` | `chess_msgs/BoardState` (authoritative) |
| pub | `/chess/perceived_state` | `chess_msgs/BoardState` (vision) |
| sub | `/overhead_camera/image` | `sensor_msgs/Image` |
| srv | `/chess/voice/parse` | `chess_msgs/ParseVoiceCommand` |
| act | `/chess/execute_move` | `chess_msgs/ExecuteChessMove` |

---

## 8. SOLID notes — where the seams are

* **SRP**: each node owns one responsibility (capture | ASR | parsing |
  rules | orchestration | motion | perception). No node does two jobs.
* **OCP**: `PieceDetector`, `MoveParser`, `ChessEngine` are Protocols. New
  backends (e.g. faster-whisper, a local Llama-cpp, a learned detector)
  plug in without changing any consumer.
* **LSP**: `RegexFallbackParser` is substitutable for `LangChainLLMParser`;
  `NullDetector` for `YoloPieceDetector`. Used in the test paths.
* **ISP**: services are narrow (`ParseVoiceCommand`, `SetGripper`,
  `AttachPiece`) rather than one fat "ChessRobotCommand".
* **DIP**: `voice_parser_node`, `game_manager`, `board_state_estimator`
  all depend on Protocols defined in `chess_brain`/`chess_voice`/
  `chess_perception`, not on concrete classes.

---

## 9. Known limitations / next steps

* The Gazebo detachable-joint plugin (`gz-sim-detachable-joint-system`) is
  exposed only via `gz` CLI here; replace `chess_motion.gazebo_attach` with
  proper Python bindings (`gz.transport13`) when they become widely
  packaged in Jazzy.
* The vision pipeline is calibrated for the simulated nadir camera. On
  real hardware, swap `IntrinsicHomography` for an ArUco-based
  `MarkerHomography` calibration.
* No game clock yet; trivially added as a thin node publishing time on
  `/chess/clock_state`.
* The LLM is consulted on every utterance. For latency, cache the last
  parse + add a small grammar fallback (already partly in
  `RegexFallbackParser`).
