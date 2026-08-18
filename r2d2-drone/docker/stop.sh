#!/usr/bin/env bash

set -e

########################################
# Configuration
########################################

CONTAINER_NAME="r2d2_drone_dev"

########################################
# Check Container
########################################

if docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then

    echo "[INFO] Stopping '${CONTAINER_NAME}'..."

    docker stop "${CONTAINER_NAME}"

    echo "[INFO] Container stopped."

else

    echo "[INFO] Container is already stopped."

fi
