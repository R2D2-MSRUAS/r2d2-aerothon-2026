#!/usr/bin/env bash

set -e

########################################
# Configuration
########################################

CONTAINER_NAME="r2d2_drone_dev"

########################################
# Check Container
########################################

if ! docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then
    echo "[ERROR] Container '${CONTAINER_NAME}' is not running."
    echo "[INFO] Run ./run.sh first."
    exit 1
fi

########################################
# Open Shell
########################################

docker exec -it "${CONTAINER_NAME}" bash
