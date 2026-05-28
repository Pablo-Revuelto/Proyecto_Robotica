# Training a chess-piece YOLOv8 model

The perception node `chess_perception.board_state_estimator` expects a
12-class YOLOv8 model with labels:

    white_pawn, white_knight, white_bishop, white_rook, white_queen, white_king,
    black_pawn, black_knight, black_bishop, black_rook, black_queen, black_king

## 1. Get a dataset

Two recommended sources (already labelled in YOLO format):

- **Roboflow Universe — "Chess Pieces Detection"**:
  https://universe.roboflow.com/joseph-nelson/chess-pieces-new
  Download in YOLOv8 format. Re-map class names to match the list above if
  needed (some datasets use 1-letter labels: K, Q, R, B, N, P with case
  encoding the colour).

- **Capture your own dataset from Gazebo**: run `chess_gazebo.launch.py`,
  then drive Gazebo to record `/overhead_camera/image` frames while the
  spawn node randomises the FEN. Use [Label Studio](https://labelstud.io/)
  or [CVAT](https://www.cvat.ai/) for annotation. Synthetic + 200 real
  frames is usually enough for the overhead-camera setup.

## 2. Train

```bash
pip install ultralytics
yolo detect train data=chess.yaml model=yolov8n.pt epochs=80 imgsz=640
```

where `chess.yaml` declares the 12-class label list above and points to
`train/`, `val/` image directories.

The training script writes the best checkpoint to
`runs/detect/train/weights/best.pt`.

## 3. Use it

Copy the resulting `best.pt` to this directory (or anywhere) and launch the
estimator with:

```bash
ros2 launch chess_bringup chess_full.launch.py \
  yolo_weights:=$(realpath best.pt)
```

The estimator publishes `/chess/perceived_state` (`chess_msgs/BoardState`),
which the game manager cross-checks against its internal engine state.
