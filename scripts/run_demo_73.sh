#!/usr/bin/env bash
#
# run_demo_73.sh — launch the 7.3 demo: voice + LLM, arm execution.
#
# Vision (YOLO) is OFF by default: it is only an advisory/experimental validator
# and is NOT reliable enough to drive the board. To watch the advisory channel,
# launch the vision variant documented in README_PROYECTO.md instead.
#
# Prerequisites (one-time):
#   colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
#   bash scripts/setup_ai_venv.sh
#   /opt/ai-venv/bin/huggingface-cli login
#
# Usage:
#   bash scripts/run_demo_73.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# --- AI bridge: make /opt/ai-venv importable by the ROS nodes (numpy<2) ------
export PYTHONPATH="/opt/ai-venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
# --- WSL2 audio: route sounddevice to the WSLg PulseAudio server -------------
export PULSE_SERVER="unix:/mnt/wslg/PulseServer"

# --- Source ROS + the workspace overlay --------------------------------------
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
    echo "ERROR: ${REPO_ROOT}/install/setup.bash not found. Build first:" >&2
    echo "  colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"

echo "[run] Launching 7.3 demo (voice + LLM, vision OFF) ..."
exec ros2 launch chess_bringup chess_full.launch.py \
    enable_voice:=true \
    enable_vision:=false
