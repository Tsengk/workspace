#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-auto_calib_melodic}"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "[into_docker] Container is not running: ${CONTAINER_NAME}"
  echo "[into_docker] Start it first: ./docker/start_docker.sh"
  exit 1
fi

xhost +local:root >/dev/null 2>&1 || true

# 进入后自动 source melodic，方便直接编译和启动 GUI
docker exec -u root -it "$CONTAINER_NAME" bash -lc "source /opt/ros/melodic/setup.bash; exec bash"

xhost -local:root >/dev/null 2>&1 || true
