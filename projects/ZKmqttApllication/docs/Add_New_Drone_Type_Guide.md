# 新增无人机装备类型开发接入指南

> 以新增类型码 `1006` 为例 | 版本：v2.0 | 日期：2026-07-30

---

## 目录

1. [核心概念](#1-核心概念)
2. [架构总览：类型码 → 控制函数 → 控制话题](#2-架构总览类型码--控制函数--控制话题)
3. [改动点全景图](#3-改动点全景图)
4. [Step 1：配置文件——注册装备 + 声明话题](#4-step-1配置文件注册装备--声明话题)
5. [Step 2：控制函数——新增独立控制函数](#5-step-2控制函数新增独立控制函数)
6. [Step 3：MQTT 路径——/llm/jsim/excute 分发](#6-step-3mqtt-路径llmjsimexcute-分发)
7. [Step 4：RPC 路径——dotask 分发](#7-step-4rpc-路径dotask-分发)
8. [Step 5：视频流、位置上报、速度配置](#8-step-5视频流位置上报速度配置可选)
9. [Step 6：任务完成通知](#9-step-6任务完成通知可选)
10. [完整改动文件清单](#10-完整改动文件清单)
11. [后续扩展：类型码映射表](#11-后续扩展类型码映射表推荐重构)

---

## 1. 核心概念

```
[ZYC]
1002_1=dy99        ← 1002是类型码，dy99是装备编号
  ↑      ↑
类型码   装备编号（zb_id）
```

| 概念 | 示例 | 含义 |
|------|------|------|
| **类型码** | `1002`、`1006` | 装备类别，决定走哪个控制函数 + 哪个 MQTT 话题 |
| **装备编号** | `dj01`、`new_drone_01` | 具体某架无人机，任务中 `model_id`/`uid` 的值 |

**核心设计原则**：每种类型码应拥有自己独立的 **控制函数** 和 **控制话题**，不复用，不耦合。

---

## 2. 架构总览：类型码 → 控制函数 → 控制话题

```
                         gkd_config.ini
                    ┌─────────────────────────┐
                    │ [ZYC]                   │
                    │ 1002_1=dj01   (Mini4)   │
                    │ 1006_1=nd01   (新型)    │
                    └──────────┬──────────────┘
                               │ loadFromIni()
                               ▼
         ┌─────────────────────────────────────────────┐
         │              zyc map (装备注册)               │
         │  zyc["1002"] = [{dj01},{dj02},...]          │
         │  zyc["1006"] = [{nd01},{nd02},...]          │
         └──────┬──────────────────┬───────────────────┘
                │                  │
   MQTT 路径   ▼                  ▼   RPC 路径
  ┌─────────────────────┐  ┌─────────────────────────┐
  │ xjsim.cpp           │  │ c2manager.cpp           │
  │ /llm/jsim/excute    │  │ RPC "dotask"            │
  │                     │  │                         │
  │ if 1002:            │  │ if 1002:                │
  │   ControlDJiMini4   │  │   ControlDJiMini4       │
  │   → ZK/PathTopic    │  │   → ZK/PathTopic        │
  │     (函数内硬编码)    │  │     (函数内硬编码)        │
  │                     │  │                         │
  │ if 1006:            │  │ if 1006:                │
  │   ControlDrone1006  │  │   ControlDrone1006      │
  │   → ZK/PathTopic1006│  │   → ZK/PathTopic1006    │
  │     (函数内硬编码)    │  │     (函数内硬编码)        │
  └─────────────────────┘  └─────────────────────────┘
```

---

## 3. 改动点全景图

| # | 文件 | 改动内容 | 优先级 |
|---|------|----------|--------|
| ① | `gkd_config.ini` | 在 `[ZYC]` 添加 `1006_N=装备编号` | **必须** |
| ② | `xjsim.h` | 声明新控制函数 `ControlDrone1006` | **必须** |
| ③ | `xjsim.cpp` | 实现 `ControlDrone1006`（含话题硬编码） + MQTT 路径分发 | **必须** |
| ④ | `c2manager.h` | 声明新控制函数 `ControlDrone1006` | **必须** |
| ⑤ | `c2manager.cpp` | 实现 `ControlDrone1006`（含话题硬编码） + RPC 路径分发 | **必须** |
| ⑥ | `xjsim.cpp:1589` | `/aircraft/location` 中文名映射 | 可选 |
| ⑦ | `xjsim.cpp:1617` | `llm/task/zc/command` 任务完成处理 | 可选 |
| ⑧ | `device_velocity_config.ini` | 速度配置 | 可选 |

---

## 4. Step 1：配置文件——注册装备

唯一需要在配置文件中做的：在 `[ZYC]` 节添加新类型装备。话题名在控制函数内硬编码，不通过配置文件声明。

### gkd_config.ini

```ini
# ============================================================
# 装备资源池
# ============================================================
[ZYC]
# 类型 1002：大疆 Mini 4 Pro（已有）
1002_1=dy99
1002_2=dy100
1002_3=dy101
1002_4=dj01
1002_5=dj02
1002_6=dj03

# 类型 1006：XXX 新型无人机（新增）
1006_1=nd01
1006_2=nd02

# 类型 2003：单兵装备（已有）
2003_1=dy20
2003_2=bd01
```

> `loadFromIni()`（`gkd_config.h:160-180`）会自动遍历 `[ZYC]` 下所有 key，按类型码分组存入 `zyc` map。**配置层无需任何代码改动。**

---

## 5. Step 2：控制函数——新增独立控制函数

每种无人机类型应有自己的控制函数，便于独立定制协议体。

### 5.1 xjsim.h —— 声明

```cpp
// Src/xjsim/xjsim.h，class XjSim，private 区域:

// 已有：
bool ControlDJiMini4_Pro(const string& rid, double executeHeight,
                         double waypointSpeed,
                         std::vector<MQTTMission::Position>& path);

// 新增：
bool ControlDrone1006(const string& rid, double executeHeight,
                      double waypointSpeed,
                      std::vector<MQTTMission::Position>& path);
```

### 5.2 xjsim.cpp —— 实现

```cpp
// Src/xjsim/xjsim.cpp，在 ControlDJiMini4_Pro 函数后面新增:

bool XjSim::ControlDrone1006(const string& rid, double executeHeight,
                              double waypointSpeed,
                              std::vector<MQTTMission::Position>& path)
{
    if (path.size() < 2) return false;

    QJsonObject root;
    addHeader(root);
    QJsonObject CTN;

    // ==========================================
    // 根据 1006 型无人机的飞控协议定制 JSON 结构
    // 以下为示例，按实际协议修改字段名和结构
    // ==========================================
    CTN["rid"] = QString::fromStdString(rid);
    CTN["wpml_executeHeight"] = executeHeight;
    CTN["wpml_waypointSpeed"] = waypointSpeed;

    QJsonArray point_list;
    for (const MQTTMission::Position& position : path) {
        QJsonObject pos;
        pos["longitude"] = position.longitude;
        pos["latitude"]  = position.latitude;
        pos["altitude"]  = position.altitude;
        // 1006 特有字段示例:
        // pos["gimbal_pitch"] = -90;
        // pos["action_type"]  = "hover";
        point_list.push_back(pos);
    }

    QJsonObject line_list;
    line_list["point_list"] = point_list;
    CTN["line_list"] = line_list;
    root["CTN"] = CTN;

    // ==========================================
    // 控制话题在函数内硬编码，和现有 1002 风格一致
    // ==========================================
    this->mqClient->GetMqttConn()->PrimitivePublish("ZK/PathTopic1006", root);

    return true;
}
```

### 5.3 c2manager.h —— 声明

```cpp
// Src/gkd/c2manager.h，class C2Manager，private 区域:

// 已有：
bool ControlDJiMini4_Pro(QString rid, double executeHeight,
                         double waypointSpeed,
                         std::vector<MQTTMission::Position>& path);

// 新增：
bool ControlDrone1006(QString rid, double executeHeight,
                      double waypointSpeed,
                      std::vector<MQTTMission::Position>& path);
```

### 5.4 c2manager.cpp —— 实现

```cpp
// Src/gkd/c2manager.cpp，在 ControlDJiMini4_Pro 函数后面新增:

bool C2Manager::ControlDrone1006(QString rid, double executeHeight,
                                  double waypointSpeed,
                                  std::vector<MQTTMission::Position>& path)
{
    if (path.size() < 2) return false;

    QJsonObject root;
    addHeader(root);
    QJsonObject CTN;

    // ==========================================
    // 1006 协议体，按实际协议定制
    // ==========================================
    CTN["rid"] = rid;
    CTN["wpml_executeHeight"] = executeHeight;
    CTN["wpml_waypointSpeed"] = waypointSpeed;

    QJsonArray point_list;
    for (MQTTMission::Position& position : path) {
        QJsonObject pos;
        pos["longitude"] = position.longitude;
        pos["latitude"]  = position.latitude;
        pos["altitude"]  = executeHeight;
        point_list.push_back(pos);
    }

    QJsonObject line_list;
    line_list["point_list"] = point_list;
    CTN["line_list"] = line_list;
    root["CTN"] = CTN;

    // 控制话题在函数内硬编码
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/PathTopic1006", root);

    _prcp_client->m_rcpClientAdapter->showNotification(
        "MQTT消息", "1006型无人机行动控制下发成功", 0, 4);

    return true;
}
```

---

## 6. Step 3：MQTT 路径——/llm/jsim/excute 分发

**文件**：`Src/xjsim/xjsim.cpp`，约第 1509-1531 行

在 `/llm/jsim/excute` 回调中，为 `1006` 新增独立的 `else if` 分支：

```cpp
// ============================================
// 1002 分支（已有，保持不变）
// ============================================
if (_gkd_cfg.zyc.find("1002") != _gkd_cfg.zyc.end() &&
    hasId(_gkd_cfg.zyc["1002"], uid.toStdString()))
{
    if ((!_llm_excute.contains("executeHeight")) ||
        (!_llm_excute.contains("waypointSpeed")))
        return;

    std::vector<MQTTMission::Position> path;
    QJsonArray data = _llm_excute["data"].toArray();
    for (QJsonValue p : data) {
        QJsonObject point = p.toObject();
        MQTTMission::Position position;
        position.longitude = QJsonValue(point["lon"]).toDouble();
        position.latitude  = QJsonValue(point["lat"]).toDouble();
        position.altitude  = QJsonValue(point["height"]).toDouble();
        path.push_back(position);
    }
    if (path.size() < 2) return;

    double executeHeight = _llm_excute["executeHeight"].toDouble();
    double waypointSpeed = _llm_excute["waypointSpeed"].toDouble();
    ControlDJiMini4_Pro(uid.toStdString(), executeHeight, waypointSpeed, path);
}
// ============================================
// 1006 分支（新增）
// ============================================
else if (_gkd_cfg.zyc.find("1006") != _gkd_cfg.zyc.end() &&
         hasId(_gkd_cfg.zyc["1006"], uid.toStdString()))
{
    if ((!_llm_excute.contains("executeHeight")) ||
        (!_llm_excute.contains("waypointSpeed")))
        return;

    std::vector<MQTTMission::Position> path;
    QJsonArray data = _llm_excute["data"].toArray();
    for (QJsonValue p : data) {
        QJsonObject point = p.toObject();
        MQTTMission::Position position;
        position.longitude = QJsonValue(point["lon"]).toDouble();
        position.latitude  = QJsonValue(point["lat"]).toDouble();
        position.altitude  = QJsonValue(point["height"]).toDouble();
        path.push_back(position);
    }
    if (path.size() < 2) return;

    double executeHeight = _llm_excute["executeHeight"].toDouble();
    double waypointSpeed = _llm_excute["waypointSpeed"].toDouble();
    ControlDrone1006(uid.toStdString(), executeHeight, waypointSpeed, path);
}
// ============================================
// 其他类型：走 planZB → QB/303（已有）
// ============================================
else if (_llm_excute["cmd"] == "RUN_UAV_UID" ||
         _llm_excute["cmd"] == "RUN_UGV_UID")
{
    // ... 保持不变
}
```

> **关键**：`1006` 用了 **独立的 `else if`** 而不是和 `1002` 共用条件。这样每个类型可以：
> - 解析不同的 JSON 字段（比如 `1006` 可能需要额外参数 `gimbal_mode`）
> - 调用不同的控制函数
> - 校验不同的参数阈值

---

## 7. Step 4：RPC 路径——dotask 分发

**文件**：`Src/gkd/c2manager.cpp`，约第 402-431 行

在 `dotask` 回调的 `behavior_list` 遍历中，为 `1006` 新增分支：

```cpp
for (int i = 0; i < behavior_list.size(); i++) {
    QJsonObject behavior = behavior_list[i].toObject();
    QString rid = behavior["model_id"].toString();

    // ============================================
    // 1002 分支（已有，保持不变）
    // ============================================
    if (_gkd_cfg->zyc.find("1002") != _gkd_cfg->zyc.end() &&
        hasId(_gkd_cfg->zyc["1002"], rid.toStdString()))
    {
        QJsonObject data = behavior["data"].toObject();
        QJsonArray movePath = data["movePath"].toArray();
        double limieHight = data["limitHeight"].toDouble();
        _mini4_pro_path.clear();

        for (int j = 0; j < movePath.size(); j++) {
            QJsonArray pos_list = movePath[j].toArray();
            MQTTMission::Position position;
            position.longitude = pos_list[0].toDouble();
            position.latitude  = pos_list[1].toDouble();
            position.altitude  = pos_list[2].toDouble();
            _mini4_pro_path.push_back(position);
        }

        ControlDJiMini4_Pro(rid, limieHight, speed, _mini4_pro_path);
    }
    // ============================================
    // 1006 分支（新增）
    // ============================================
    else if (_gkd_cfg->zyc.find("1006") != _gkd_cfg->zyc.end() &&
             hasId(_gkd_cfg->zyc["1006"], rid.toStdString()))
    {
        QJsonObject data = behavior["data"].toObject();
        QJsonArray movePath = data["movePath"].toArray();
        double limitHeight = data["limitHeight"].toDouble();

        // 1006 使用独立的路径缓存，不复用 _mini4_pro_path
        std::vector<MQTTMission::Position> drone1006_path;
        for (int j = 0; j < movePath.size(); j++) {
            QJsonArray pos_list = movePath[j].toArray();
            MQTTMission::Position position;
            position.longitude = pos_list[0].toDouble();
            position.latitude  = pos_list[1].toDouble();
            position.altitude  = pos_list[2].toDouble();
            drone1006_path.push_back(position);
        }

        // 1006 可以有独立的 speed 或从配置读取
        double speed1006 = 10;  // 或 _gkd_cfg->getDeviceVelocity(rid);
        ControlDrone1006(rid, limitHeight, speed1006, drone1006_path);
    }
    // ============================================
    // 2003 单兵分支（已有，保持不变）
    // ============================================
    else if (_gkd_cfg->zyc.find("2003") != _gkd_cfg->zyc.end() &&
             hasId(_gkd_cfg->zyc["2003"], rid.toStdString()))
    {
        // ... 保持不变
    }
}
```

> 也要注意 RPC 回调外层的高度校验逻辑（c2manager.cpp:380-398），如果 1006 有不同的高度约束，需要一并调整。

---

## 8. Step 5：视频流、位置上报、速度配置（可选）

### 8.1 视频流

无需改动。`videoStreamCall` 回调（`c2manager.cpp:490-516`）基于装备编号动态拼接：

```
rtmp://192.168.43.220:1935/live/nd01
```

### 8.2 位置上报中文名

`Src/xjsim/xjsim.cpp:1589-1592`，追加映射：

```cpp
if (uid == "nd01") uname = "1006型无人机1";
if (uid == "nd02") uname = "1006型无人机2";
```

### 8.3 速度配置

`device_velocity_config.ini`：

```ini
[DeviceTaskVelocity]
nd01_220=12
nd01_221=12

[DefaultVelocity]
nd01=10
nd02=10
```

---

## 9. Step 6：任务完成通知（可选）

**文件**：`Src/xjsim/xjsim.cpp:1617-1622`

当前订阅 `llm/task/zc/command` 处理 `data == "dj_finish"` 时清除大疆路线标绘。如果 1006 型无人机也有同样的需求，追加处理：

```cpp
// 原代码：
else if (t_data == "dj_finish") {
    // 重置路线标绘
    for (auto uuid : _task_plane_roate_uuid_list) {
        this->rcp_->m_rcpClientAdapter->deleteMark(uuid);
    }
    _task_plane_roate_uuid_list.clear();
}
// 新增：
else if (t_data == "drone1006_finish") {
    for (auto uuid : _task_plane_roate_uuid_list_1006) {
        this->rcp_->m_rcpClientAdapter->deleteMark(uuid);
    }
    _task_plane_roate_uuid_list_1006.clear();
}
```

同时在 `xjsim.h` 新增成员：
```cpp
std::vector<string> _task_plane_roate_uuid_list_1006;
```

---

## 10. 完整改动文件清单

| # | 文件 | 改动性质 | 优先级 |
|---|------|----------|--------|
| ① | `gkd_config.ini` | 在 `[ZYC]` 添加 `1006_N=装备编号` | **必须** |
| ② | `Src/xjsim/xjsim.h` | 声明 `ControlDrone1006()` | **必须** |
| ③ | `Src/xjsim/xjsim.cpp` | 实现 `ControlDrone1006()` + MQTT `/llm/jsim/excute` 分发分支 | **必须** |
| ④ | `Src/gkd/c2manager.h` | 声明 `ControlDrone1006()` | **必须** |
| ⑤ | `Src/gkd/c2manager.cpp` | 实现 `ControlDrone1006()` + RPC `dotask` 分发分支 | **必须** |
| ⑥ | `Src/xjsim/xjsim.cpp:1589` | `/aircraft/location` 中文名映射 | 可选 |
| ⑦ | `Src/xjsim/xjsim.cpp:1617` | `llm/task/zc/command` 任务完成处理 | 可选 |
| ⑧ | `device_velocity_config.ini` | 添加速度配置 | 可选 |

---

## 11. 后续扩展：类型码映射表（推荐重构）

当前设计在 `xjsim.cpp` 和 `c2manager.cpp` 的每个分支中有较多重复代码（JSON 解析 + hasId 调用）。如果未来需要添加更多类型（`1007`、`1008`……），建议将分发逻辑抽取为映射表：

```cpp
// 建议新建 Src/common/drone_type_registry.h

struct DroneTypeConfig {
    std::string type_code;          // "1002", "1006" ...
    std::string control_topic;      // MQTT 控制话题，硬编码
    double default_speed;           // 默认航速
    double min_height;              // 最小高度约束 (RPC 路径用)
    // 控制函数通过函数指针或 std::function 绑定
};

inline std::map<std::string, DroneTypeConfig> DRONE_TYPE_REGISTRY = {
    {"1002", {"1002", "ZK/PathTopic",      8,  20}},
    {"1006", {"1006", "ZK/PathTopic1006", 10,  30}},
};
```

这样分发逻辑可简化为查表：

```cpp
// xjsim.cpp /llm/jsim/excute:
auto cfg = DRONE_TYPE_REGISTRY.find(type_code);
if (cfg != DRONE_TYPE_REGISTRY.end() && hasId(zyc[type_code], uid)) {
    ControlDroneByType(cfg->second, uid, ...);
}
```

新增类型只需在注册表中加一行，无需修改 `xjsim.cpp` 和 `c2manager.cpp` 的分支逻辑。

---

> **总结**：每个新无人机类型 = 独立的控制话题 + 独立的控制函数 + 独立的 MQTT/RPC 分发分支。当前文档的设计保证了各类无人机之间的零耦合，后续可按此模板快速复制。
