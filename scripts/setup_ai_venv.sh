#!/usr/bin/env bash
#
# setup_ai_venv.sh — create /opt/ai-venv and install the 7.3 AI/voice/vision
# dependencies (requirements-ai.txt) WITHOUT touching the system Python.
#
# Safe to re-run. Does NOT install or store any token. After it finishes, log in
# to Hugging Face with `huggingface-cli login` (printed at the end).
#
# Usage:
#   bash scripts/setup_ai_venv.sh
#
set -euo pipefail

# Resolve the repo root from this script's location (works from any CWD).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REQS="${REPO_ROOT}/requirements-ai.txt"

VENV="/opt/ai-venv"
# CPU-only PyTorch wheels (avoids dragging in CUDA on a laptop/WSL2).
TORCH_CPU_INDEX="https://download.pytorch.org/whl/cpu"

if [[ ! -f "${REQS}" ]]; then
    echo "ERROR: ${REQS} not found. Run this from inside the repo." >&2
    exit 1
fi

# --- 1. Create the venv (needs sudo only the first time, just for /opt) -------
if [[ ! -d "${VENV}" ]]; then
    echo "[setup] Creating virtualenv at ${VENV} ..."
    if [[ -w /opt ]]; then
        python3 -m venv "${VENV}"
    else
        echo "[setup] /opt is not writable; creating ${VENV} with sudo and"
        echo "        chowning it to $(id -un) so later pip needs no sudo."
        sudo python3 -m venv "${VENV}"
        sudo chown -R "$(id -un):$(id -gn)" "${VENV}"
    fi
else
    echo "[setup] Reusing existing virtualenv at ${VENV}"
fi

PY="${VENV}/bin/python"
PIP="${PY} -m pip"

# Sanity: the PYTHONPATH bridge expects python3.12 site-packages.
PYVER="$(${PY} -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${PYVER}" != "3.12" ]]; then
    echo "[setup] WARNING: venv Python is ${PYVER}, but the bridge path used by"
    echo "        run_demo_73.sh is .../python3.12/site-packages. Adjust the"
    echo "        PYTHONPATH in scripts/run_demo_73.sh to python${PYVER}."
fi

# --- 2. Install dependencies -------------------------------------------------
echo "[setup] Upgrading pip ..."
${PIP} install --upgrade pip

echo "[setup] Installing requirements (CPU torch index as fallback) ..."
${PIP} install --extra-index-url "${TORCH_CPU_INDEX}" -r "${REQS}"

# --- 3. Re-pin numpy<2 LAST (protects ROS/cv_bridge) -------------------------
echo "[setup] Enforcing numpy<2 (ROS/cv_bridge requirement) ..."
${PIP} install "numpy<2"

echo
echo "[setup] Installed versions:"
${PIP} list 2>/dev/null | grep -iE "^numpy|^torch|^transformers|^sounddevice|^langchain|^huggingface|^ultralytics|^opencv" || true

echo
echo "==============================================================="
echo " AI venv ready at ${VENV}."
echo
echo " NEXT: authenticate with Hugging Face (token is NOT stored in Git):"
echo
echo "     ${VENV}/bin/huggingface-cli login"
echo
echo " Paste a Read access token from https://huggingface.co/settings/tokens"
echo " It is saved to ~/.cache/huggingface/token and read automatically."
echo "==============================================================="
