#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="${CONTAINER_NAME:-auto_calib_melodic}"
IMG="${IMG:-osrf/ros:melodic-desktop-full}"

USER_ID="$(id -u)"
GRP_ID="$(id -g)"
USER_NAME="${USER:-$(id -un)}"

DISPLAY_VAR="${DISPLAY:-:0}"
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"

if [ ! -f "$XAUTH_FILE" ]; then
  touch "$XAUTH_FILE"
fi

mkdir -p "$HOME/.cache"

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
  docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
fi

xhost +local:root >/dev/null 2>&1 || true

echo "[start_docker] CONTAINER_NAME=${CONTAINER_NAME}"
echo "[start_docker] IMG=${IMG}"
echo "[start_docker] WS_ROOT=${WS_ROOT}"
echo "[start_docker] DISPLAY=${DISPLAY_VAR}"

docker run -it -d \
  --name "$CONTAINER_NAME" \
  --privileged \
  --net=host \
  --shm-size=1g \
  -e DISPLAY="$DISPLAY_VAR" \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/root/.Xauthority \
  -e DOCKER_USER="$USER_NAME" \
  -e DOCKER_USER_ID="$USER_ID" \
  -e DOCKER_GRP_ID="$GRP_ID" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH_FILE":/root/.Xauthority:rw \
  -v "$HOME/.cache":/root/.cache:rw \
  -v /etc/localtime:/etc/localtime:ro \
  -v "$WS_ROOT":/ws:rw \
  -w /ws \
  "$IMG" \
  bash -lc "source /opt/ros/melodic/setup.bash; exec bash"

if ! docker exec "$CONTAINER_NAME" bash -lc 'dpkg -s libceres-dev >/dev/null 2>&1'; then
  echo "[start_docker] Installing Ceres in container..."
  docker exec -u root "$CONTAINER_NAME" bash -lc 'export DEBIAN_FRONTEND=noninteractive; apt-get update && apt-get install -y libceres-dev'
fi

echo "[start_docker] Container started."
echo "[start_docker] Enter with: ./docker/into_docker.sh"
echo "[start_docker] Build in container: source /opt/ros/melodic/setup.bash && catkin_make -j"
