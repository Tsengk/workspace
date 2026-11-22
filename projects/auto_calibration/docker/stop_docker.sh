#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-auto_calib_melodic}"

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
  docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
  echo "[stop_docker] Removed container: ${CONTAINER_NAME}"
else
  echo "[stop_docker] Container not found: ${CONTAINER_NAME}"
fi
