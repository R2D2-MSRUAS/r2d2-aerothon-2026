#!/usr/bin/env bash

set -e

########################################
# Configuration
########################################

IMAGE_NAME="r2d2_drone:jazzy"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

########################################
# Build Image
########################################

echo "========================================"
echo " Building Docker Image"
echo "========================================"

docker build \
    -t "${IMAGE_NAME}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo ""
echo "[SUCCESS] Docker image built successfully."
echo ""
echo "Image Name : ${IMAGE_NAME}"
