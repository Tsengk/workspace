#!/usr/bin/env bash
set -euo pipefail

# 运行时打包说明：
# 1. 打包工作区内所有 ROS 包的运行所需内容。
# 2. 打包时通过临时 install 空间收敛运行文件，避免把 src/ 源码目录带进交付物。
# 3. 目标机前提固定为 Ubuntu 20.04 + ROS Noetic + 图形桌面环境。

usage() {
  cat <<'EOF'
用法：
  bash scripts/package_auto_calibration_runtime.sh [选项]

选项：
  --ros-setup <path>     ROS setup.bash 路径，默认 /opt/ros/noetic/setup.bash
  --dist-dir <path>      输出目录，默认 <workspace>/dist
  --bundle-name <name>   运行包语义化名称，默认 auto_calibration_runtime
  --build-dir <path>     临时构建目录，默认 /tmp/<bundle_name>_build
  --install-dir <path>   临时安装目录，默认 /tmp/<bundle_name>_install
  --skip-build           跳过重新编译，直接复用 install-dir
  --archive-only         打包完成后删除输出目录，仅保留 tar.gz
  -h, --help             显示帮助

说明：
  1. 默认会重新编译，因为需要先生成干净 install 空间，再做运行时打包。
  2. 运行包默认输出为：
     <workspace>/dist/auto_calibration_runtime/
     <workspace>/dist/auto_calibration_runtime.tar.gz
  3. 默认会同时保留目录和压缩包，便于本机直接验证。
  4. 如果只想保留压缩包，可加 --archive-only。
  5. 脚本会覆盖同名运行包目录和压缩包。
EOF
}

log() {
  printf '[package_auto_calibration_runtime] %s\n' "$*"
}

fail() {
  printf '[package_auto_calibration_runtime] ERROR: %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "缺少命令: $1"
}

is_elf_file() {
  local path="$1"
  [[ -f "${path}" ]] || return 1
  file -Lb "${path}" | grep -q '^ELF'
}

should_bundle_dep() {
  local path="$1"
  local base
  base="$(basename "${path}")"

  [[ -f "${path}" ]] || return 1

  case "${path}" in
    "${INSTALL_DIR}/"*|\
    "${BUNDLE_DIR}/install/"*|\
    "${BUNDLE_DIR}/lib/"*|\
    "${BUNDLE_DIR}/qt_plugins/"*|\
    /opt/ros/*)
      return 1
      ;;
  esac

  case "${base}" in
    linux-vdso.so.*|\
    ld-linux-*.so*|ld-*.so|\
    libc.so.*|libc-*.so|\
    libm.so.*|libm-*.so|\
    libpthread.so.*|libpthread-*.so|\
    libdl.so.*|libdl-*.so|\
    librt.so.*|librt-*.so|\
    libutil.so.*|libutil-*.so|\
    libresolv.so.*|libresolv-*.so|\
    libnsl.so.*|libnsl-*.so|\
    libcrypt.so.*|libcrypt-*.so)
      return 1
      ;;
  esac

  return 0
}

parse_ldd_paths() {
  local target="$1"
  LD_LIBRARY_PATH="${BUNDLE_DIR}/lib:${BUNDLE_DIR}/install/lib:${BUNDLE_DIR}/install/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}" \
  ldd "${target}" 2>/dev/null | awk '
    /=>/ {
      if ($3 == "not") {
        printf("UNRESOLVED %s\n", $1) > "/dev/stderr";
        exit 42;
      }
      if ($3 ~ /^\//) print $3;
      next;
    }
    /^[[:space:]]*\/[^[:space:]]+/ {
      print $1;
    }' | sort -u
}

copy_qt_plugins() {
  local qt_plugin_root
  qt_plugin_root="$(qmake -query QT_INSTALL_PLUGINS)"
  [[ -d "${qt_plugin_root}" ]] || fail "Qt 插件目录不存在: ${qt_plugin_root}"

  local rel
  local plugins=(
    "platforms/libqxcb.so"
    "imageformats/libqjpeg.so"
    "imageformats/libqsvg.so"
    "iconengines/libqsvgicon.so"
    "xcbglintegrations/libqxcb-glx-integration.so"
  )

  for rel in "${plugins[@]}"; do
    local src="${qt_plugin_root}/${rel}"
    local dst="${BUNDLE_DIR}/qt_plugins/${rel}"
    if [[ -f "${src}" ]]; then
      mkdir -p "$(dirname "${dst}")"
      cp -fL "${src}" "${dst}"
    fi
  done
}

write_launcher() {
  cat > "${BUNDLE_DIR}/run_auto_calibration_gui.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 环境脚本不存在: ${ROS_SETUP}" >&2
  exit 1
fi

# ROS Noetic 的部分 catkin env hook 在 ROS_DISTRO 未预先定义时与 nounset 不兼容，
# 这里临时关闭 set -u，避免目标机仅因环境变量未预置而启动失败。
set +u
source "${ROS_SETUP}"
export _CATKIN_SETUP_DIR="${SELF_DIR}/install"
source "${SELF_DIR}/install/setup.bash"
set -u
unset _CATKIN_SETUP_DIR

# 优先使用随包动态库和 Qt 插件，减少目标机对三方库安装状态的依赖。
export LD_LIBRARY_PATH="${SELF_DIR}/lib:${SELF_DIR}/install/lib:${SELF_DIR}/install/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="${SELF_DIR}/qt_plugins"
export QT_QPA_PLATFORM_PLUGIN_PATH="${SELF_DIR}/qt_plugins/platforms"

# 覆盖 Mesa GL/GLSL 版本，兼容软件渲染场景下的 Qt 3D 加速需求
export MESA_GL_VERSION_OVERRIDE=3.2
export MESA_GLSL_VERSION_OVERRIDE=150

exec roslaunch calibration_gui calibration_gui.launch "$@"
EOF
  chmod +x "${BUNDLE_DIR}/run_auto_calibration_gui.sh"
}

write_readme() {
  cat > "${BUNDLE_DIR}/README.txt" <<'EOF'
auto_calibration_runtime

目标机前提：
1. Ubuntu 20.04 x86_64
2. 已安装 ROS Noetic
3. 具备图形桌面环境

启动方式：
1. 进入当前目录
2. 执行：
   ./run_auto_calibration_gui.sh

说明：
1. 运行包包含工作区内所有 ROS 包（标定、感知、传感器驱动等）
2. 当前 GUI 实际开放能力仍由包内 config/tabs_config.yaml 控制
3. install/ 是安装后的 ROS 运行空间
4. lib/ 是随包带走的非 ROS 运行库
5. qt_plugins/ 是 GUI 所需的 Qt 插件
6. 本包仅包含程序运行所需内容
EOF
}

verify_runtime() {
  local target
  local verify_ld_path
  verify_ld_path="${BUNDLE_DIR}/lib:${BUNDLE_DIR}/install/lib:${BUNDLE_DIR}/install/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

  while IFS= read -r target; do
    is_elf_file "${target}" || continue
    if LD_LIBRARY_PATH="${verify_ld_path}" ldd "${target}" | grep -q 'not found'; then
      LD_LIBRARY_PATH="${verify_ld_path}" ldd "${target}" >&2 || true
      fail "存在未解析依赖: ${target}"
    fi
  done < <(find "${BUNDLE_DIR}/install/lib" "${BUNDLE_DIR}/lib" "${BUNDLE_DIR}/qt_plugins" -type f | sort)

  # 这里仅做 launch 解析验证，不直接启动 GUI，避免在无 DISPLAY 场景下误失败。
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  export _CATKIN_SETUP_DIR="${BUNDLE_DIR}/install"
  # shellcheck disable=SC1091
  source "${BUNDLE_DIR}/install/setup.bash"
  unset _CATKIN_SETUP_DIR
  export LD_LIBRARY_PATH="${verify_ld_path}"
  export QT_PLUGIN_PATH="${BUNDLE_DIR}/qt_plugins"
  export QT_QPA_PLATFORM_PLUGIN_PATH="${BUNDLE_DIR}/qt_plugins/platforms"

  # 验证所有包可被 rospack 发现
  local pkg
  local pkg_list
  pkg_list=$(printf '%s\n' "${PACKAGES}" | tr ';' '\n')
  while IFS= read -r pkg; do
    [[ -n "${pkg}" ]] || continue
    rospack find "${pkg}" >/dev/null
  done <<< "${pkg_list}"

  # 验证各包 launch 文件可解析
  local pkg
  local launch_failed=0
  local pkg_list
  pkg_list=$(printf '%s\n' "${PACKAGES}" | tr ';' '\n')
  while IFS= read -r pkg; do
    [[ -n "${pkg}" ]] || continue
    local pkg_share
    pkg_share="$(rospack find "${pkg}" 2>/dev/null)" || continue
    local launch_dir="${pkg_share}/launch"
    if [[ -d "${launch_dir}" ]]; then
      local lf
      while IFS= read -r lf; do
        if ! roslaunch --nodes "${pkg}" "$(basename "${lf}")" >/dev/null 2>&1; then
          log "WARNING: launch 文件解析失败: ${pkg}/$(basename "${lf}")"
          launch_failed=1
        fi
      done < <(find "${launch_dir}" -maxdepth 1 -name '*.launch' | sort)
    fi
  done <<< "${pkg_list}"
  if [[ ${launch_failed} -ne 0 ]]; then
    log "WARNING: 部分 launch 文件存在解析问题，但 rospack 依赖已验证通过"
  fi
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS_SETUP="/opt/ros/noetic/setup.bash"
DIST_DIR="${WORKSPACE_DIR}/dist"
BUNDLE_NAME="auto_calibration_runtime"
SKIP_BUILD=0
ARCHIVE_ONLY=0
PACKAGES="bev_calibration;bev_surround_view;calibration_gui;camera_model;camera_model_test;image_lidar_calibration;lidar_camera_projection;lidar_camera_projection_bev;lidar_gnss_yaw;lidar_lidar_cali;lidar_rp_cali;lidar_secondary_rp_cali;lidar_yaw1;multi_view_stitch;radar_cali;sensor_sim_pub;vehicle_config;devkit_shm_param;gassd_msgs;gassd_msgs_gsds;chengtech_radar2cloud;hw_fr671_offline;hw_l106_offline;hw_l306_offline;tanway_focus_t_offline;tanway_scope128_offline"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-setup)
      [[ $# -ge 2 ]] || fail "--ros-setup 缺少参数"
      ROS_SETUP="$2"
      shift 2
      ;;
    --dist-dir)
      [[ $# -ge 2 ]] || fail "--dist-dir 缺少参数"
      DIST_DIR="$2"
      shift 2
      ;;
    --bundle-name)
      [[ $# -ge 2 ]] || fail "--bundle-name 缺少参数"
      BUNDLE_NAME="$2"
      shift 2
      ;;
    --build-dir)
      [[ $# -ge 2 ]] || fail "--build-dir 缺少参数"
      BUILD_DIR="$2"
      shift 2
      ;;
    --install-dir)
      [[ $# -ge 2 ]] || fail "--install-dir 缺少参数"
      INSTALL_DIR="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --archive-only)
      ARCHIVE_ONLY=1
      shift
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

BUILD_DIR="${BUILD_DIR:-/tmp/${BUNDLE_NAME}_build}"
INSTALL_DIR="${INSTALL_DIR:-/tmp/${BUNDLE_NAME}_install}"
BUNDLE_DIR="${DIST_DIR}/${BUNDLE_NAME}"
ARCHIVE_PATH="${DIST_DIR}/${BUNDLE_NAME}.tar.gz"

require_cmd cmake
require_cmd tar
require_cmd qmake
require_cmd ldd
require_cmd file
require_cmd awk
require_cmd rospack
require_cmd roslaunch

[[ -d "${WORKSPACE_DIR}/src" ]] || fail "工作区不存在: ${WORKSPACE_DIR}"
[[ -f "${ROS_SETUP}" ]] || fail "ROS 环境脚本不存在: ${ROS_SETUP}"

log "输出目录: ${DIST_DIR}"
log "运行包名称: ${BUNDLE_NAME}"
log "临时构建目录: ${BUILD_DIR}"
log "临时安装目录: ${INSTALL_DIR}"

# 运行包属于可再生构建产物，脚本默认刷新同名输出，便于后续重复打包。
rm -rf "${BUNDLE_DIR}" "${ARCHIVE_PATH}" "${BUILD_DIR}"
if [[ ${SKIP_BUILD} -eq 0 ]]; then
  rm -rf "${INSTALL_DIR}"
fi

mkdir -p "${DIST_DIR}" "${BUNDLE_DIR}/install" "${BUNDLE_DIR}/lib" "${BUNDLE_DIR}/qt_plugins"

if [[ ${SKIP_BUILD} -eq 0 ]]; then
  mkdir -p "${BUILD_DIR}" "${INSTALL_DIR}"
  log "配置 catkin 工作区"
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  cmake -S "${WORKSPACE_DIR}/src" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCATKIN_ENABLE_TESTING=OFF \
    -DCATKIN_WHITELIST_PACKAGES="${PACKAGES}"

  log "单线程构建并安装运行包"
  cmake --build "${BUILD_DIR}" --target install
else
  [[ -f "${INSTALL_DIR}/setup.bash" ]] || fail "--skip-build 模式下 install-dir 无效: ${INSTALL_DIR}"
  log "跳过编译，直接复用 install 空间: ${INSTALL_DIR}"
fi

log "复制 install 运行空间"
(cd "${INSTALL_DIR}" && tar cf - \
  --exclude='./include' \
  --exclude='./lib/pkgconfig' \
  --exclude='./share/*/cmake' \
  .) | (cd "${BUNDLE_DIR}/install" && tar xf -)

# 嵌套包 (sensor_driver/*) 在 install 空间展平为 share/<pkg>/，
# 兼容历史配置里的 sensor_driver/<pkg>/... 路径写法。
# 为每个子包单独创建 symlink，避免整目录 symlink 导致 rospack 递归循环。
mkdir -p "${BUNDLE_DIR}/install/share/sensor_driver"
if [[ -d "${WORKSPACE_DIR}/src/sensor_driver" ]]; then
  while IFS= read -r _pkg_dir; do
    _pkg="$(basename "${_pkg_dir}")"
    if [[ -d "${BUNDLE_DIR}/install/share/${_pkg}" ]]; then
      ln -snf "../${_pkg}" "${BUNDLE_DIR}/install/share/sensor_driver/${_pkg}"
    fi
  done < <(find "${WORKSPACE_DIR}/src/sensor_driver" -mindepth 1 -maxdepth 1 -type d | sort)
fi

log "复制 Qt 插件"
copy_qt_plugins

log "收集非 ROS 运行库"
declare -A COPIED_LIBS=()
declare -A QUEUED=()
declare -a QUEUE=()

while IFS= read -r target; do
  is_elf_file "${target}" || continue
  target="$(readlink -f "${target}")"
  if [[ -z "${QUEUED[${target}]:-}" ]]; then
    QUEUED["${target}"]=1
    QUEUE+=("${target}")
  fi
done < <(find "${BUNDLE_DIR}/install/lib" "${BUNDLE_DIR}/qt_plugins" -type f | sort)

for ((idx=0; idx<${#QUEUE[@]}; ++idx)); do
  current="${QUEUE[$idx]}"
  while IFS= read -r dep; do
    [[ -n "${dep}" ]] || continue
    dep="$(readlink -f "${dep}")"
    if ! should_bundle_dep "${dep}"; then
      continue
    fi
    if [[ -n "${COPIED_LIBS[${dep}]:-}" ]]; then
      continue
    fi
    cp -fL "${dep}" "${BUNDLE_DIR}/lib/"
    COPIED_LIBS["${dep}"]=1
    if [[ -z "${QUEUED[${dep}]:-}" ]]; then
      QUEUED["${dep}"]=1
      QUEUE+=("${dep}")
    fi
  done < <(parse_ldd_paths "${current}")
done

log "生成启动脚本和说明文件"
write_launcher
write_readme

log "验证运行时闭包"
verify_runtime

log "生成压缩包: ${ARCHIVE_PATH}"
tar -C "${DIST_DIR}" -czf "${ARCHIVE_PATH}" "${BUNDLE_NAME}"

if [[ ${ARCHIVE_ONLY} -eq 1 ]]; then
  log "按 archive-only 模式删除输出目录: ${BUNDLE_DIR}"
  rm -rf "${BUNDLE_DIR}"
fi

log "打包完成"
log "压缩包: ${ARCHIVE_PATH}"
if [[ ${ARCHIVE_ONLY} -eq 0 ]]; then
  log "运行包目录: ${BUNDLE_DIR}"
fi
