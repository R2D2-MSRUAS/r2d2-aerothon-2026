#!/usr/bin/env bash

set -e

########################################
# Configuration
########################################

CONTAINER_NAME="r2d2_drone_dev"
IMAGE_NAME="r2d2_drone:jazzy"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

HOST_SRC="${PROJECT_ROOT}/src"

CONTAINER_WS="/root/drone_ws"
CONTAINER_SRC="${CONTAINER_WS}/src"

########################################
# X11 Access
########################################

xhost +SI:localuser:root >/dev/null

########################################
# Create src if missing
########################################

mkdir -p "${HOST_SRC}"

########################################
# GPU Detection
########################################

GPU_ARGS=""

if command -v nvidia-smi >/dev/null 2>&1; then
    if docker info | grep -qi "nvidia"; then
        GPU_ARGS="--gpus all"
        echo "[INFO] NVIDIA GPU acceleration enabled."
    else
        echo "[WARNING] NVIDIA GPU detected, but NVIDIA Container Toolkit is not installed."
    fi
else
    echo "[INFO] No NVIDIA GPU detected. Running without GPU acceleration."
fi

########################################
# Existing Container
########################################

if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then

    echo "[INFO] Starting existing container..."

    docker start "${CONTAINER_NAME}" >/dev/null

    docker exec -it "${CONTAINER_NAME}" bash

    exit 0

fi

########################################
# Create New Container
########################################

echo "[INFO] Creating new container..."

docker run -it \
    --name "${CONTAINER_NAME}" \
    --hostname r2d2-drone \
    --network host \
    --ipc=host \
    --privileged \
    ${GPU_ARGS} \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${HOST_SRC}:${CONTAINER_SRC}:rw" \
    -w "${CONTAINER_WS}" \
    "${IMAGE_NAME}"
