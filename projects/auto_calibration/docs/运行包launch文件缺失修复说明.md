# 运行包 launch 文件缺失修复说明

## 1. 问题现象

使用运行包脚本：

`/opt/usr/ap_adsfi/tools/auto_calibration_runtime/run_auto_calibration_gui.sh`

启动时出现错误：

`RLException: .../sensor_driver/hw_l106_offline/launch/offline_front.launch is not a launch file name`

导致离线雷达点云无法正常拉起与显示。

## 2. 根因分析

本问题由以下三点叠加导致：

1. 部分 `sensor_driver` 包未安装 `launch/` 目录与节点可执行文件到 runtime `install/` 目录。
2. `calibration_gui` 对配置中的 launch 路径解析过于依赖固定相对路径，安装目录结构变化时容易失效。
3. 运行包打包脚本只为少量包创建 `share/sensor_driver/<pkg>` 映射，未覆盖全部 `sensor_driver` 子包。

## 3. 代码修改内容

### 3.1 补齐 sensor_driver 安装规则

在以下文件中补充安装规则，安装目标包括：

- 可执行文件安装到 `${CATKIN_PACKAGE_BIN_DESTINATION}`
- `launch/` 目录安装到 `${CATKIN_PACKAGE_SHARE_DESTINATION}/launch`

修改文件：

- `src/sensor_driver/hw_l106_offline/CMakeLists.txt`
- `src/sensor_driver/hw_fr671_offline/CMakeLists.txt`
- `src/sensor_driver/hw_l306_offline/CMakeLists.txt`
- `src/sensor_driver/chengtech_radar2cloud/CMakeLists.txt`
- `src/sensor_driver/tanway_focus_t_offline/CMakeLists.txt`
- `src/sensor_driver/tanway_scope128_offline/CMakeLists.txt`

新增代码模式（6 个包一致）：

```cmake
install(TARGETS <node_name>
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
```

### 3.2 增强 launch 路径解析容错

修改文件：

- `src/calibration_gui/src/config_manager.cpp`

改动点：

- 新增 `resolveLaunchPath(...)` 路径解析逻辑。
- 若配置为绝对路径，直接使用。
- 优先尝试原有 `configDir()/../../ + launch_file` 方式。
- 若文件不存在且路径以 `sensor_driver/` 开头，则自动去掉该前缀再解析一次，兼容 `share/<pkg>/launch` 结构。

关键代码：

```cpp
namespace {

bool fileExists(const std::string& path)
{
    std::ifstream f(path);
    return f.good();
}

std::string resolveLaunchPath(const std::string& config_dir,
                              const std::string& launch_file)
{
    if (launch_file.empty()) return "";
    if (launch_file[0] == '/') return launch_file;

    const std::string base = config_dir + "/../../";
    const std::string direct = base + launch_file;
    if (fileExists(direct)) return direct;

    static const std::string kSensorDriverPrefix = "sensor_driver/";
    if (launch_file.compare(0, kSensorDriverPrefix.size(), kSensorDriverPrefix) == 0) {
        const std::string fallback = base + launch_file.substr(kSensorDriverPrefix.size());
        if (fileExists(fallback)) return fallback;
    }
    return direct;
}

} // namespace
```

调用点修改：

```cpp
return resolveLaunchPath(configDir(), l.launch_file);
return resolveLaunchPath(configDir(), r.launch_file);
```

### 3.3 打包脚本支持全部 sensor_driver 子包

修改文件：

- `scripts/package_auto_calibration_runtime.sh`

改动点：

- 从“仅处理少量硬编码包”改为“遍历 `src/sensor_driver` 下全部子包”。
- 自动创建 `install/share/sensor_driver/<pkg> -> ../<pkg>` 软链接映射。

关键代码：

```bash
mkdir -p "${BUNDLE_DIR}/install/share/sensor_driver"
if [[ -d "${WORKSPACE_DIR}/src/sensor_driver" ]]; then
  while IFS= read -r _pkg_dir; do
    _pkg="$(basename "${_pkg_dir}")"
    if [[ -d "${BUNDLE_DIR}/install/share/${_pkg}" ]]; then
      ln -snf "../${_pkg}" "${BUNDLE_DIR}/install/share/sensor_driver/${_pkg}"
    fi
  done < <(find "${WORKSPACE_DIR}/src/sensor_driver" -mindepth 1 -maxdepth 1 -type d | sort)
fi
```

## 4. 代码改动明细文件

为便于 code review，完整原始差异已保存：

- `docs/运行包launch文件缺失修复.diff`

## 5. 运行包产物更新

已重新生成并验证以下产物：

- `dist/auto_calibration_runtime/`
- `dist/auto_calibration_runtime.tar.gz`

关键文件存在性已确认：

- `install/share/hw_l106_offline/launch/offline_front.launch`
- `install/share/sensor_driver/hw_l106_offline/launch/offline_front.launch`
- `install/lib/hw_l106_offline/hw_l106_offline`

## 6. 实际部署路径同步

已将最新运行包部署到：

`/opt/usr/ap_adsfi/tools/auto_calibration_runtime`

部署后 `offline_front.launch` 已可在上述路径找到，原始报错对应文件缺失问题已消除。

## 7. 备注

在受限环境下执行 `roslaunch` 可能出现 `~/.ros` 写入告警或网卡权限相关告警，这类告警与本次 `offline_front.launch` 缺失问题无关。

## 8. 对非打包运行环境的影响

这里的“非打包运行环境”指源码工作区直接运行（例如 `source devel/setup.bash` 后启动）。

影响结论如下：

1. `src/calibration_gui/src/config_manager.cpp` 的路径解析改动会生效，但对现有源码直跑行为通常无破坏性变化。  
   逻辑是“先按原路径找，再按兼容路径兜底”，主要提升容错。
2. `src/sensor_driver/*/CMakeLists.txt` 新增的 `install()` 规则只影响 install 空间与打包产物完整性。  
   不执行 install/打包时，不改变 devel 空间运行逻辑。
3. `scripts/package_auto_calibration_runtime.sh` 的修改仅在执行该脚本时生效。  
   不打包场景下不会触发，也不会影响源码运行。

因此，本次修复对“不打包直接运行”是兼容的，重点收益在“install/运行包”链路。
