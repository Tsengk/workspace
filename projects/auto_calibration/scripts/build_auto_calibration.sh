#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法：
  bash scripts/build_auto_calibration.sh [选项]

选项：
  --ros-setup <path>    ROS 环境脚本路径，默认 /opt/ros/noetic/setup.bash
  --workspace <path>    catkin 工作区根目录，默认脚本上一级目录
  --build-dir <path>    指定构建目录，默认按工作区路径生成独立目录
  -h, --help            显示帮助

说明：
  1. 编译工作区内所有 ROS 包（通过 --force-cmake 重新配置）。
  2. 脚本固定使用 catkin_make 单线程编译，不会触发并行构建。
  3. 默认不会复用工作区根目录下的 build/，避免工程拷贝或改名后命中旧的 CMakeCache。
EOF
}

log() {
  printf '[build_auto_calibration] %s\n' "$*"
}

fail() {
  printf '[build_auto_calibration] ERROR: %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "缺少命令: $1"
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS_SETUP="/opt/ros/noetic/setup.bash"
BUILD_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-setup)
      [[ $# -ge 2 ]] || fail "--ros-setup 缺少参数"
      ROS_SETUP="$2"
      shift 2
      ;;
    --workspace)
      [[ $# -ge 2 ]] || fail "--workspace 缺少参数"
      WORKSPACE_DIR="$2"
      shift 2
      ;;
    --build-dir)
      [[ $# -ge 2 ]] || fail "--build-dir 缺少参数"
      BUILD_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "未知参数: $1"
      ;;
  esac
done

require_cmd catkin_make
require_cmd awk
require_cmd md5sum

[[ -d "${WORKSPACE_DIR}/src" ]] || fail "不是有效的 catkin 工作区: ${WORKSPACE_DIR}"
[[ -f "${ROS_SETUP}" ]] || fail "ROS 环境脚本不存在: ${ROS_SETUP}"

if [[ -z "${BUILD_DIR}" ]]; then
  # 构建目录按工作区绝对路径分桶，避免拷贝工程后复用旧路径生成的 CMakeCache。
  WORKSPACE_HASH=$(printf '%s' "${WORKSPACE_DIR}" | md5sum | awk '{print substr($1, 1, 10)}')
  BUILD_DIR="${WORKSPACE_DIR}/.build_auto_calibration_${WORKSPACE_HASH}"
fi

log "工作区: ${WORKSPACE_DIR}"
log "ROS 环境: ${ROS_SETUP}"
log "构建目录: ${BUILD_DIR}"

# catkin_make 会优先读取 MAKEFLAGS；这里显式清掉外部并行参数，确保最终始终为单线程。
unset MAKEFLAGS || true
export ROS_PARALLEL_JOBS='-j1 -l1'

# shellcheck disable=SC1090
source "${ROS_SETUP}"

cd "${WORKSPACE_DIR}"

log "开始执行 catkin_make（全工作区编译）"

catkin_make \
  --build "${BUILD_DIR}" \
  --force-cmake \
  --cmake-args \
    -DCATKIN_ENABLE_TESTING=OFF \
    -DCATKIN_WHITELIST_PACKAGES=""

log "编译完成"
log "工作区内所有 ROS 包已编译"
