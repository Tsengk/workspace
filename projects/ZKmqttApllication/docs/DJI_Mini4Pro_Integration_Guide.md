# 大疆 Mini 4 Pro 无人机系统接入开发文档

> 版本：v1.0 | 日期：2026-07-30
>
> 云台控制补充

---

## 目录

1. [系统概览](#1-系统概览)
2. [架构总览](#2-架构总览)
3. [注册与配置](#3-注册与配置)
4. [任务路径下发（控制链路）](#4-任务路径下发控制链路)
5. [视频流接入](#5-视频流接入)
6. [遥测与位置上报](#6-遥测与位置上报)
7. [关键数据结构](#7-关键数据结构)
8. [配置文件参考](#8-配置文件参考)
9. [新增无人机装备步骤](#9-新增无人机装备步骤)
10. [常见问题排查](#10-常见问题排查)

---

## 1. 系统概览

DJI Mini 4 Pro 在系统中作为**实体无人机**管理，区别于仿真无人机。核心能力：

| 能力 | 说明 |
|------|------|
| **任务路径下发** | 通过 MQTT `ZK/PathTopic` 主题下发航线（经纬度+高度+速度） |
| **视频流拉取** | RTMP 协议，地址动态拼接：`rtmp://{ip}:1935/live/{装备编号}` |
| **实时位置上报** | 订阅 `/aircraft/location` 获取经纬度/高度 |

**关键标识**：在资源池 `[ZYC]` 中注册为类型码 `1002`（大疆无人机类）。

---

## 2. 架构总览

```
┌────────────────────────────────────────────────────────────────────────┐
│                           国遥3D引擎 (RPC Server)                        │
│  提供: taskplan / dotask / videoStreamCall / locatesmsi 等RPC方法        │
└────────────────────────────┬───────────────────────────────────────────┘
                             │ RPC (rest_rpc)
                             ▼
┌────────────────────────────────────────────────────────────────────────┐
│                     ZKmqttApplication (RPC Client)                      │
│                                                                         │
│  ┌─────────────┐     ┌──────────────────┐     ┌──────────────────┐      │
│  │  C2Manager   │     │     XjSim        │     │   mainwindow     │      │
│  │              │     │                  │     │                  │      │
│  │ RPC回调:     │     │ MQTT回调:        │     │ 初始化:          │      │
│  │ dotask ──────┼─────┼──► ZK/PathTopic  │     │ _gkd_cfg.load()  │      │
│  │ taskplan     │     │ /llm/jsim/excute │     │ RPC连接建立      │      │
│  │ videoStream  │     │ /aircraft/loc    │     │ C2Manager创建    │      │
│  └──────┬───────┘     └────────┬─────────┘     └──────────────────┘      │
│         │                      │                                        │
└─────────┼──────────────────────┼────────────────────────────────────────┘
          │                      │
          │ MQTT (mosquitto)     │ MQTT
          ▼                      ▼
┌────────────────────────────────────────────────────────────────────────┐
│                          MQTT Broker                                    │
│                                                                         │
│  发布主题:                 订阅主题:                                      │
│  ZK/PathTopic  (航线)     /aircraft/location  (无人机位置)                │
│  ZK/101        (规划请求)  /llm/jsim/excute    (行动控制)                 │
│  ZK/103        (任务计划)  llm/task/zc/command (任务完成)                 │
│  ZK/104        (平台计划)                                               │
└────────────────────────────────────────────────────────────────────────┘
          │
          ▼
┌────────────────────────────────────────────────────────────────────────┐
│                        大疆 Mini 4 Pro 飞控系统                          │
│                                                                         │
│  - 订阅 ZK/PathTopic 接收航线                                            │
│  - 发布 /aircraft/location 上报位置                                      │
│  - 推送 RTMP 视频流到 rtmpserver                                         │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 注册与配置

### 3.1 资源池注册 `[ZYC]`

系统通过 `gkd_config.ini` 的 `[ZYC]` 节管理装备清单。类型码 `1002` = 大疆无人机类。

```ini
[ZYC]
1002_1=dy99
1002_2=dy100
1002_3=dy101
1002_4=dj01
1002_5=dj02
1002_6=dj03
2003_1=dy20
2003_2=bd01
```

**解析逻辑** (`Src/common/gkd_config.h:160-180`)：

```
Key 拆分: "1002_1" → zc_type="1002", value="dy99"
            ↑  ↓
       类型码  序号（仅用于去重，不作为索引）
```

加载后存入 `GKDCommonCfg::zyc`：
```cpp
std::map<std::string, std::vector<zy>> zyc;
// zyc["1002"] = [ {"dy99",""}, {"dy100",""}, {"dy101",""},
//                 {"dj01",""}, {"dj02",""}, {"dj03",""} ]
// zyc["2003"] = [ {"dy20",""}, {"bd01",""} ]
```

### 3.2 数据结构

```cpp
// Src/common/gkd_config.h:6-9
struct zy {
    string zb_id;   // 装备编号（如 "dj01"），用于身份匹配
    string name;    // 装备名称（如 "无人机1"），通过 dotask 动态更新
};
```

### 3.3 身份验证机制 —— hasId()

整个系统的装备身份校验由 `hasId()` 函数完成。两个模块（xjsim、c2manager）各自实现了**完全相同的** `hasId()`，因为它们是独立的类，没有共享这个工具函数。

#### 3.3.1 源码

**XjSim 版本** — `Src/xjsim/xjsim.cpp:1246-1254`：

```cpp
bool XjSim::hasId(const std::vector<zy>& vec, const std::string& model_id)
{
    auto it = std::find_if(vec.begin(), vec.end(),
                           [&model_id](const zy& item) {
                               return item.zb_id == model_id; // 只比较 zb_id 字段
                           });

    return it != vec.end();
}
```

**C2Manager 版本** — `Src/gkd/c2manager.cpp:3439-3446`：

```cpp
bool C2Manager::hasId(const std::vector<zy>& vec, const std::string& model_id) {
    auto it = std::find_if(vec.begin(), vec.end(),
                           [&model_id](const zy& item) {
                               return item.zb_id == model_id; // 只比较 zb_id 字段
                           });

    return it != vec.end();
}
```

#### 3.3.2 函数签名解析

```
bool hasId(
    const std::vector<zy>& vec,    // ① 在哪找：某个类型码下的全部装备列表
    const std::string& model_id    // ② 找什么：任务中携带的装备编号
);
```

| 参数 | 实际传入值 | 来源 |
|------|-----------|------|
| `vec` | `_gkd_cfg.zyc["1002"]` | `gkd_config.ini → [ZYC] → loadFromIni()` |
| `model_id` | 任务中的 `uid` 或 `model_id` | MQTT `/llm/jsim/excute` 的 `uid` 字段 / RPC `dotask` 的 `behavior.model_id` |

#### 3.3.3 逐行解析

**第 1 步：`std::find_if` 线性遍历**

```cpp
auto it = std::find_if(vec.begin(), vec.end(), ...);
```

`std::find_if` 是 C++ 标准库算法，从 `vec.begin()` 到 `vec.end()` 逐个元素调用 lambda，**找到第一个返回 `true` 的元素就停止**，返回指向该元素的迭代器；全部不满足则返回 `vec.end()`。

时间复杂度 O(n)，n = 该类型码下的装备数量（通常不超过几十个）。

**第 2 步：lambda 精确字符串比较**

```cpp
[&model_id](const zy& item) {
    return item.zb_id == model_id;
}
```

- `[&model_id]`：按引用捕获外层的 `model_id` 参数
- `const zy& item`：当前遍历到的 `zy` 结构体
- `item.zb_id == model_id`：**只比较 `zb_id` 字段**，忽略 `name` 字段
- 使用 `std::string::operator==`，**大小写敏感**，逐字符精确匹配

**第 3 步：返回布尔结果**

```cpp
return it != vec.end();
```

- `it != vec.end()` → 找到了，返回 `true` → 身份验证通过
- `it == vec.end()` → 没找到，返回 `false` → 身份验证失败，任务被**静默丢弃**

#### 3.3.4 执行示例

假设 `gkd_config.ini` 配置如下：

```ini
[ZYC]
1002_1=dj01
1002_2=dj02
1002_3=dj03
```

加载后内存中：

```
zyc["1002"] = [
    { zb_id: "dj01", name: "" },
    { zb_id: "dj02", name: "" },
    { zb_id: "dj03", name: "" },
]
```

**场景 A：匹配成功**

```
hasId(zyc["1002"], "dj02")

遍历:
  item[0].zb_id = "dj01"  ==  "dj02" ?  ✗
  item[1].zb_id = "dj02"  ==  "dj02" ?  ✓  ← 匹配！停止遍历
  item[2] 不再检查

返回值: true  → 身份验证通过 → 执行 ControlDJiMini4_Pro
```

**场景 B：匹配失败（常见 bug）**

```
hasId(zyc["1002"], "di02")

遍历:
  item[0].zb_id = "dj01"  ==  "di02" ?  ✗
  item[1].zb_id = "dj02"  ==  "di02" ?  ✗  ← 注意：d'j'02 vs d'i'02
  item[2].zb_id = "dj03"  ==  "di02" ?  ✗

返回值: false  → 身份验证失败 → 任务被丢弃，ZK/PathTopic 无消息
```

**场景 C：未注册的装备**

```
hasId(zyc["1002"], "dj04")

遍历:
  item[0].zb_id = "dj01"  ==  "dj04" ?  ✗
  item[1].zb_id = "dj02"  ==  "dj04" ?  ✗
  item[2].zb_id = "dj03"  ==  "dj04" ?  ✗

返回值: false  → 身份验证失败 → 任务被丢弃
```

#### 3.3.5 调用上下文

`hasId()` 总是在两层检查中被调用：

```cpp
// 形式（以 xjsim.cpp 为例）:
if (_gkd_cfg.zyc.find("1002") != _gkd_cfg.zyc.end() &&   // ① 类型码 "1002" 是否存在
    hasId(_gkd_cfg.zyc["1002"], uid.toStdString()))       // ② 装备编号是否在 "1002" 列表中
{
    // 两条都满足 → 执行大疆 Mini 4 Pro 控制逻辑
}

// 外层 find 的作用：
//   如果 [ZYC] 中完全没有 "1002" 这个类型码，直接短路，
//   避免访问不存在的 zyc["1002"]（map::operator[] 会创建空条目）
```

完整调用链如下：

```
gkd_config.ini [ZYC] 节
       │
       ▼
GKDCommonCfg::loadFromIni()
       │ 解析 key "1002_1=dj01" → zc_type="1002", value="dj01"
       │ 存入 zyc["1002"].push_back({"dj01", ""})
       │
       ▼
zyc map 就绪:
  zyc["1002"] = [{zb_id:"dj01",...}, {zb_id:"dj02",...}, ...]
       │
       ├── xjsim.cpp:1513 ── hasId(zyc["1002"], uid) ──► 控制 DJI Mini 4 Pro
       │
       └── c2manager.cpp:405 ── hasId(zyc["1002"], rid) ──► 控制 DJI Mini 4 Pro
```

#### 3.3.6 两个 hasId() 为何重复

| | XjSim::hasId | C2Manager::hasId |
|---|---|---|
| 所在文件 | `Src/xjsim/xjsim.cpp:1246` | `Src/gkd/c2manager.cpp:3439` |
| 调用方 | xjsim 的 MQTT 回调 | c2manager 的 RPC 回调 |
| 输入 vec | `_gkd_cfg.zyc["1002"]` | `_gkd_cfg->zyc["1002"]` |
| 逻辑 | 完全相同 | 完全相同 |

两者是**不同类的成员函数**，没有继承关系，所以各自实现了一份。实际上可以抽取到 `gkd_config.h` 中作为 `GKDCommonCfg` 的成员（如方案 B 的 `isDjiDrone()`），但目前代码未做此重构。

#### 3.3.7 常见坑

| 问题 | 现象 | 原因 |
|------|------|------|
| 大小写不一致 | 任务中 `DJ01`，配置中 `dj01` | `"DJ01" == "dj01"` 为 `false` |
| 拼写错误 | `1002_5=di02`（应为 `dj02`） | `"di02" == "dj02"` 为 `false`，差一个字符 |
| 类型码写错 | 装备配在 `[ZYC]` 的 `1003_1=dj01` | `zyc.find("1002")` 返回 `end`，外层短路 |
| 中英文标点混入 | 配置行末尾有多余空格或不可见字符 | `"dj01 " == "dj01"` 为 `false` |

**所有这些问题都不会报错，任务被静默丢弃，MQTT 无任何输出。排查时需对比配置文件和任务消息中的字符串是否逐字符一致。**

---

## 4. 任务路径下发（控制链路）——代码级详细梳理

DJI Mini 4 Pro 有**两条独立的任务下发路径**，最终都通过 MQTT `ZK/PathTopic` 话题将航线发送给大疆飞控。

```
                         任务来源
                    ┌────────┴────────┐
                    ▼                  ▼
          MQTT /llm/jsim/excute    RPC "dotask" (国遥引擎)
                    │                  │
                    ▼                  ▼
              xjsim.cpp:1509      c2manager.cpp:354
              XjSim 回调           C2Manager dt lambda
                    │                  │
            ┌───────┴───────┐  ┌──────┴───────┐
            │ hasId 门禁    │  │ hasId 门禁    │
            │ zyc["1002"]   │  │ zyc["1002"]   │
            └───────┬───────┘  └──────┬───────┘
                    │                  │
                    ▼                  ▼
         XjSim::                C2Manager::
         ControlDJiMini4_Pro    ControlDJiMini4_Pro
         (xjsim.cpp:2565)       (c2manager.cpp:3407)
                    │                  │
                    └────────┬─────────┘
                             ▼
                   MQTT ZK/PathTopic
                   (大疆飞控订阅)
```

---

### 4.1 路径 A：MQTT `/llm/jsim/excute` (xjsim)

**文件**：`Src/xjsim/xjsim.cpp`  
**所属函数**：`XjSim::InitMqttAndRcp()`

#### 4.1.1 入口：回调注册 (行 1509)

```cpp
this->mqClient->GetMqttConn()->Subscribe(
    QString::fromStdString("/llm/jsim/excute"),
    [this](const QJsonObject& root) {
        // ... 处理逻辑
    });
```

- `Subscribe()` 底层基于 mosquitto 库
- 每当 MQTT Broker 上有 `/llm/jsim/excute` 消息到达，lambda 被回调
- `root` 是已经反序列化好的 `QJsonObject`

#### 4.1.2 第 1 步：缓存原始消息 + 提取 uid (行 1511-1512)

```cpp
_llm_excute = root;  // 存到成员变量，checkDeviceStatus 重试时复用
QString uid = _llm_excute["uid"].toString();
```

#### 4.1.3 第 2 步：第一层门禁——类型码 + 装备编号双重校验 (行 1513-1514)

```cpp
if(_gkd_cfg.zyc.find("1002") != _gkd_cfg.zyc.end() &&
            hasId(_gkd_cfg.zyc["1002"],uid.toStdString()) ){
```

两级短路求值：

```
① zyc.find("1002") != zyc.end()
   ├─ false → 整个 if 为 false，跳过（[ZYC] 中没有 1002 这个类型码）
   └─ true  → 继续执行 ②

② hasId(zyc["1002"], uid)
   ├─ false → 整个 if 为 false，进入 else if 仿真路径
   └─ true  → 进入分支，执行大疆 Mini 4 Pro 控制逻辑
```

> 注释中保留了旧版硬编码 `// if(uid == "dy99" || uid == "dy100" || uid == "dy101")`，现已废弃。

#### 4.1.4 第 3 步：第二层门禁——必填字段校验 (行 1516)

```cpp
if((!_llm_excute.contains("executeHeight"))||
   (!_llm_excute.contains("waypointSpeed")))
    return;    // 静默丢弃
```

#### 4.1.5 第 4 步：解析路径点 (行 1518-1527)

```cpp
std::vector<MQTTMission::Position> path;
QJsonArray data = _llm_excute["data"].toArray();
for(QJsonValue p: data){
    QJsonObject point = p.toObject();
    MQTTMission::Position position;
    position.longitude = QJsonValue(point["lon"]).toDouble();
    position.latitude  = QJsonValue(point["lat"]).toDouble();
    position.altitude  = QJsonValue(point["height"]).toDouble();
    path.push_back(position);
}
```

输入 JSON 格式——**对象数组**：

```json
{
  "uid": "dj01",
  "executeHeight": 120.0,
  "waypointSpeed": 15.0,
  "data": [
    {"lon": 106.5178, "lat": 29.7300, "height": 120.0},
    {"lon": 106.5180, "lat": 29.7305, "height": 150.0}
  ]
}
```

| JSON 字段 | 目标字段 | 说明 |
|-----------|----------|------|
| `data[].lon` | `position.longitude` | 经度 |
| `data[].lat` | `position.latitude` | 纬度 |
| `data[].height` | `position.altitude` | **保留原始值**，各点高度可以不同 |

#### 4.1.6 第 5 步：第三层门禁——最少路径点数 (行 1528)

```cpp
if(path.size() < 2) return;   // 少于 2 个点无法构成航线 → 丢弃
```

#### 4.1.7 第 6 步：先发布云台命令，再发布航线

```cpp
double executeHeight = _llm_excute["executeHeight"].toDouble();
double waypointSpeed = _llm_excute["waypointSpeed"].toDouble();
ControlDJiMini4_Pro_Gimbal_pitch(uid);
ControlDJiMini4_Pro(uid.toStdString(), executeHeight, waypointSpeed, path);
```

当前入口先通过资源池 `1002` 类型及 UID 匹配，再检查必填任务字段和最少两个航点，满足条件后执行上述调用。云台控制仅接入 XjSim 的 MQTT 实机分支，未接入 C2Manager 的 RPC `dotask` 下发路径，也不用于非 `1002` 装备的仿真分支。

##### 云台 MQTT 命令

实现函数：`Src/xjsim/xjsim.cpp` 中的 `XjSim::ControlDJiMini4_Pro_Gimbal_pitch()`。

控制端发布 `ZK/GimbalTopic`，Topic 大小写敏感、不带前导 `/`。报文为 UTF-8 JSON，复用当前发布接口，QoS 2、Retain=false：

```json
{
  "type": "gimbal_pitch",
  "rid": "fz03",
  "command_id": "gimbal-fz03-1794710358479",
  "mode": "absolute",
  "pitch": -90,
  "gimbal_index": "LEFT_OR_MAIN"
}
```

| 字段 | 当前赋值与含义 |
|---|---|
| `type` | 固定 `gimbal_pitch`，控制 Pitch 轴 |
| `rid` | 任务 UID 去除首尾空白，须匹配 Android 配置；空 UID 不发送 |
| `command_id` | `gimbal-{UID}-{当前Unix毫秒时间戳}`，用于设备端命令去重 |
| `mode` | 固定 `absolute`，每次设置目标角度，不累计相对旋转 |
| `pitch` | 固定 `-90` 度，实际支持范围和物理方向需机型联调确认 |
| `gimbal_index` | 固定 `LEFT_OR_MAIN`，主云台 |

请求未携带可选 `timestamp`，设备跳过命令过期检查；`command_id` 内的时间只是标识内容，不代替该协议字段。

##### 当前执行语义

```text
有效的实机任务
    → 发布 ZK/GimbalTopic（absolute，-90°）
    → 立即发布 ZK/PathTopic
```

#### 4.1.8 第 7 步：XjSim::ControlDJiMini4_Pro — 打包发布 (行 2565-2591)

```cpp
bool XjSim::ControlDJiMini4_Pro(
    const string& rid, double executeHeight, double waypointSpeed,
    std::vector<MQTTMission::Position>& path)
{
    if(path.size() < 2) return false;                     // ① 再次校验

    QJsonObject root;
    addHeader(root);  // → SID="ZK", TM=毫秒时间戳

    QJsonObject CTN;
    CTN["rid"] = QString::fromStdString(rid);
    CTN["wpml_executeHeight"] = executeHeight;
    CTN["wpml_waypointSpeed"] = waypointSpeed;

    QJsonObject line_list;
    QJsonArray point_list;
    for(MQTTMission::Position position : path){
        QJsonObject pos;
        pos["longitude"] = position.longitude;
        pos["latitude"]  = position.latitude;
        pos["altitude"]  = position.altitude;  // ← 使用各点原始高度
        point_list.push_back(pos);
    }
    line_list["point_list"] = point_list;
    CTN["line_list"] = line_list;
    root["CTN"] = CTN;

    this->mqClient->GetMqttConn()->PrimitivePublish("ZK/PathTopic", root);
    // (通知弹窗被注释掉)
    return true;
}
```

#### 4.1.9 else if 分支：非 1002 装备走仿真路径 (行 1533-1574)

```cpp
else if(_llm_excute["cmd"]=="RUN_UAV_UID" || _llm_excute["cmd"]=="RUN_UGV_UID"){
    // 构造 excuteUAVOrUGV → 路径插值 → planZB → 逐点发 QB/303（仿真无人机/车）
}
```

---

### 4.2 路径 B：RPC `dotask` (c2manager)

**文件**：`Src/gkd/c2manager.cpp`  
**所属函数**：`C2Manager::RPCSubscribe()`

#### 4.2.1 入口：RPC 订阅注册 (行 278, 354, 471)

```cpp
void C2Manager::RPCSubscribe()
{
    TPCallback dt = [this](string_view& task){
        // ... 处理逻辑 ...
    };
    _prcp_client->subscribeTask("dotask", dt);  // 行 471
}
```

**调用链**：
```
国遥引擎 RPC call "dotask"
    → rest_rcp_client::subscribeTask()
        → RcpClientAdapter::subscribeTask()
            → rpc_client::subscribe("dotask", "", cb)  // rest_rpc 库
                → dt lambda 被回调，task = 国遥引擎传来的 JSON 字符串 (string_view)
```

#### 4.2.2 第 1 步：JSON 解析 (行 358-362)

```cpp
QJsonParseError jsonError;
QJsonDocument _document = QJsonDocument::fromJson(task.data(), &jsonError);
if (jsonError.error != QJsonParseError::NoError) return;  // 格式错误 → 丢弃
QJsonObject json = _document.object();
```

#### 4.2.3 第 2 步：更新装备名称 (行 364-371)

```cpp
QJsonArray entity_model_array = json["entity_model"].toArray();
for(const QJsonValue & obj : entity_model_array){
    QJsonObject entity_model = obj.toObject();
    QString model_id = entity_model["model_id"].toString();
    QString name     = entity_model["name"].toString();
    updata_zyc_name(model_id.toStdString(), name.toStdString());
}
```

`updata_zyc_name()` 遍历 `zyc` 中**所有类型码**，找到 `zb_id` 匹配的装备，将国遥引擎传来的中文名写入 `zy.name` 字段，供位置上报（`QB/303`）时使用。

#### 4.2.4 第 3 步：提取 behavior_list (行 376-378)

```cpp
QJsonArray behavior_list;
if(json.contains("behavior_list"))
    behavior_list = json["behavior_list"].toArray();
if(behavior_list.size() == 0) return;
```

#### 4.2.5 第 4 步：高度合规校验——全量预检 (行 380-399)

```cpp
bool unroul = false;
std::vector<double> limitHeigh_list;
for(int i=0; i < behavior_list.size(); i++){
    QJsonObject behavior = behavior_list[i].toObject();
    QString rid = behavior["model_id"].toString();

    // 跳过单兵装备（2003 不走无人机高度校验）
    if(_gkd_cfg->zyc.find("2003") != _gkd_cfg->zyc.end() &&
            hasId(_gkd_cfg->zyc["2003"], rid.toStdString())){
        continue;
    }

    QJsonObject data = behavior["data"].toObject();
    double limieHight = data["limitHeight"].toDouble();
    if(limieHight < 20) unroul = true;       // 任一无人机高度 < 20m → 标记
    limitHeigh_list.push_back(limieHight);
}

// 任意两架高度差 < 10m 或 存在 < 20m → 拒绝
if(!checkAnyDifference(limitHeigh_list, 10) || unroul){
    _prcp_client->m_rcpClientAdapter->showNotification(
        "消息", "无人机高度不合规，检查高度是否小于20或者高度差小于10", 2, 4);
    return;    // 弹窗提示后拒绝
}
```

`checkAnyDifference()` 是 O(n²) 双重循环：任一配对 `|h_i - h_j| < 10` → 返回 false。

| 条件 | 结果 |
|------|------|
| 任一无人机 `limitHeight < 20` | **拒绝**，弹窗 |
| 任意两架 `|h_i - h_j| < 10` | **拒绝**，弹窗 |

#### 4.2.6 第 5 步：甘特图控制 (行 400-401)

```cpp
_prcp_client->m_rcpClientAdapter->schemeEndPlay();
_prcp_client->m_rcpClientAdapter->schemeStartPlay();
```

#### 4.2.7 第 6 步：遍历 behavior，按类型码分发 (行 402-431)

```cpp
for(int i=0; i < behavior_list.size(); i++){
    QJsonObject behavior = behavior_list[i].toObject();
    QString rid = behavior["model_id"].toString();

    // ===== 分支 1：大疆 Mini 4 Pro (1002) =====
    if(_gkd_cfg->zyc.find("1002") != _gkd_cfg->zyc.end() &&
       hasId(_gkd_cfg->zyc["1002"], rid.toStdString()))
    {
        QJsonObject data = behavior["data"].toObject();
        QJsonArray movePath = data["movePath"].toArray();
        double limieHight = data["limitHeight"].toDouble();
        _mini4_pro_path.clear();

        for(int i = 0; i < movePath.size(); i++){
            QJsonArray pos_list = movePath[i].toArray();
            MQTTMission::Position position;
            position.longitude = pos_list[0].toDouble();
            position.latitude  = pos_list[1].toDouble();
            position.altitude  = pos_list[2].toDouble();
            _mini4_pro_path.push_back(position);
        }

        ControlDJiMini4_Pro(rid, limieHight, speed, _mini4_pro_path);
        //                            ↑ 成员变量 speed = 8 (硬编码)
    }
    // ===== 分支 2：单兵装备 (2003) =====
    else if(_gkd_cfg->zyc.find("2003") != _gkd_cfg->zyc.end() &&
            hasId(_gkd_cfg->zyc["2003"], rid.toStdString()))
    {
        QJsonObject missionPlan;
        PackageMissionPlan_ToCK(json, missionPlan);
        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/103", missionPlan);
    }
}
```

**路径点格式差异**：RPC 路径用**二维数组**而非对象数组：

```json
// RPC dotask behavior 格式:
{
  "model_id": "dj01",
  "data": {
    "limitHeight": 120.0,
    "movePath": [
      [106.5178, 29.7300, 120.0],    // [lon, lat, alt]
      [106.5180, 29.7305, 120.0]
    ]
  }
}
```

#### 4.2.8 第 7 步：C2Manager::ControlDJiMini4_Pro — 打包发布 (行 3407-3432)

```cpp
bool C2Manager::ControlDJiMini4_Pro(
    QString rid, double executeHeight, double waypointSpeed,
    std::vector<MQTTMission::Position>& path)
{
    if(path.size() < 2) return false;                     // ① 再次校验

    QJsonObject root;
    addHeader(root);  // → SID="ZK", TM=毫秒时间戳

    QJsonObject CTN;
    CTN["wpml_executeHeight"] = executeHeight;
    CTN["wpml_waypointSpeed"] = waypointSpeed;
    CTN["rid"] = rid;

    QJsonObject line_list;
    QJsonArray point_list;
    for(MQTTMission::Position position : path){
        QJsonObject pos;
        pos["longitude"] = position.longitude;
        pos["latitude"]  = position.latitude;
        pos["altitude"]  = executeHeight;  // ← 用统一的 executeHeight 覆盖！
        point_list.push_back(pos);
    }
    line_list["point_list"] = point_list;
    CTN["line_list"] = line_list;
    root["CTN"] = CTN;

    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/PathTopic", root);
    _prcp_client->m_rcpClientAdapter->showNotification(
        "MQTT消息", "无人机行动控制下发成功", 0, 4);
    return true;
}
```

#### 4.2.9 第 8 步：统一发布 ZK/103 (行 457-469)

在 `behavior_list` 遍历之后，**不论是否有 1002 装备**，都会执行：

```cpp
QJsonObject missionPlan;
PackageMissionPlan_ToCK(json, missionPlan);
bool isPublish = _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/103", missionPlan);

if(isPublish){
    _prcp_client->m_rcpClientAdapter->schemeEndPlay();
    _prcp_client->m_rcpClientAdapter->schemeStartPlay();
}
```

- `ZK/103` 是作战任务计划，发给操控席位
- 再次驱动甘特图

---

### 4.3 ZK/PathTopic 最终消息格式

两条路径最终发布的 MQTT 消息结构相同：

```json
{
  "SID": "ZK",
  "TM": 1761127714551,
  "CTN": {
    "rid": "dj01",
    "wpml_executeHeight": 120.0,
    "wpml_waypointSpeed": 15.0,
    "line_list": {
      "point_list": [
        {"longitude": 106.5178, "latitude": 29.7300, "altitude": 120.0},
        {"longitude": 106.5180, "latitude": 29.7305, "altitude": 120.0}
      ]
    }
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `SID` | string | 发送方标识，固定 `"ZK"` |
| `TM` | int64 | 毫秒时间戳 |
| `CTN.rid` | string | 装备编号，大疆飞控识别目标无人机 |
| `CTN.wpml_executeHeight` | double | 执行高度（m） |
| `CTN.wpml_waypointSpeed` | double | 航点速度（m/s） |
| `CTN.line_list.point_list` | array | 航线点序列 |
| `point_list[].longitude` | double | 经度 |
| `point_list[].latitude` | double | 纬度 |
| `point_list[].altitude` | double | 高度（m） |

---

### 4.4 两条 ControlDJiMini4_Pro 实现的逐行对比

```
         XjSim 版 (xjsim.cpp:2565)           C2Manager 版 (c2manager.cpp:3407)
        ─────────────────────────           ──────────────────────────────

参数:    const string& rid                   QString rid
        double executeHeight                double executeHeight
        double waypointSpeed                double waypointSpeed
        vector<Position>& path              vector<Position>& path

①       if(path.size() < 2) return false    if(path.size() < 2) return false
        ✓ 相同                               ✓ 相同

②       addHeader(root)                     addHeader(root)
        ✓ 相同                               ✓ 相同

③       CTN["rid"] = rid;                   CTN["wpml_executeHeight"] = ...;
        CTN["wpml_executeHeight"] = ...;     CTN["wpml_waypointSpeed"] = ...;
        CTN["wpml_waypointSpeed"] = ...;     CTN["rid"] = rid;
        (字段顺序不同，JSON 无序无影响)

④       for(...) {                           for(...) {
          pos["altitude"] = position.altitude;  pos["altitude"] = executeHeight;
        }                                      }
        ← 保留各点原始高度                    ← 统一覆盖为 executeHeight

⑤       PrimitivePublish(                    PrimitivePublish(
          "ZK/PathTopic", root);               "ZK/PathTopic", root);
        ✓ 相同                               ✓ 相同

⑥       (无通知)                             showNotification(
                                               "无人机行动控制下发成功");
```

**核心差异——高度处理**：

| | xjsim 路径 | RPC dotask 路径 |
|---|---|---|
| 高度来源 | `data[].height`（每点可不同） | `behavior.data.limitHeight`（全局） |
| 各点 altitude | **保留原始值** | **统一替换为 executeHeight** |
| 适用场景 | 精细高度控制（先爬升再平飞） | 恒定高度巡航 |

---

### 4.5 门禁汇总

一条任务数据要成功到达 `ZK/PathTopic`，需依次通过以下门禁：

```
                     MQTT 路径                      RPC 路径
                     ─────────                      ────────

第一层 ── JSON 格式   (MQTT 框架保证)    ──►  QJsonDocument::fromJson 解析成功

第二层 ── 类型码      zyc.find("1002")   ──►  zyc->find("1002")
         存在性        != end                    != end

第三层 ── 装备编号    hasId(zyc["1002"], ──►  hasId(zyc["1002"],
         合法性        uid)                      rid)

第四层 ── 必填字段    executeHeight       ──►  behavior.data.limitHeight
                     + waypointSpeed            (隐式必需)

第五层 ── 路径点数    path.size() >= 2    ──►  path.size() >= 2

第六层 ── 高度合规    (无此校验)          ──►  limitHeight >= 20
                                                  且所有无人机高度差 >= 10

第七层 ── 最终出口    ZK/PathTopic        ──►  ZK/PathTopic
```

任一门禁失败 = 任务被**静默丢弃**（RPC 路径的高度校验有弹窗提示，其余全部静默）。

---

## 5. 视频流接入

### 5.1 视频流请求（国遥 RPC → 拉流）

**代码位置**：`Src/gkd/c2manager.cpp:489-516`

```
国遥引擎 RPC "videoStreamCall"
       │
       ▼
C2Manager::RPCSubscribe() → dt3 lambda
       │
       ├─ 参数: task = 装备编号字符串 (如 "dj01")
       │
       ├─ 从 gkd_config.ini 读取 rtmpserver_ip
       │
       ├─ 动态拼接视频地址:
       │   vs.vsPath = "rtmp://{rtmpserver_ip}:1935/live/{buc}"
       │
       ├─ 构造 VideoStream:
       │   vs.vsName  = "dj01[无人机dj01]"
       │   vs.model_id = "dj01"
       │   vs.equipmentSeqNo = "dj01"
       │   vs.vsType = "rtmp"
       │
       └─ 新线程调用 loadVideoStream(vslist) → RPC 推送视频流给国遥引擎
```

### 5.2 RTMP 服务器配置

```ini
# gkd_config.ini
[GKD_VIDEO]
rtmpserver_ip=192.168.43.220
```

**视频地址模板**：
```
rtmp://192.168.43.220:1935/live/{装备编号}
```

### 5.3 CK/504 视频拉流（操控席位上报）

**代码位置**：`Src/gkd/c2manager.cpp:2291-2335`

操控席位通过 MQTT `CK/504` 上报设备视频地址，C2Manager 仅加载 `video_battle_unit_code` 匹配的视频流。

---

## 6. 遥测与位置上报

### 6.1 /aircraft/location 订阅

**代码入口**：`Src/xjsim/xjsim.cpp` 中的 `XjSim::InitMqttAndRcp()`。

```
大疆飞控 → MQTT /aircraft/location
       │
       ▼
XjSim::InitMqttAndRcp() 中的 lambda
       │
       ├─ 解析 CSV 格式: "uid,lon,lat,alt"
       │   例如: "dj01,106.5178,29.7300,120.5"
       │
       ├─ 中文名映射: /llm/jsim/status/task_info 中
       │   CTN.platform_list 的 id → name，未匹配时使用 "无人机"
       │
       ├─ running_status: 根据该 UID 是否收到执行指令及 task_info.TM 更新判断
       │
       ├─ executeHeight = 10.0 + alt (补偿10m)
       │
       └─ 打包 QB/303 消息 → PrimitivePublish("QB/303", ...)
            用于更新国遥引擎中无人机模型的位置
```

装备状态规则仅作用于 `/aircraft/location` 的 `QB/303` 位置上报：
- 默认 `idle`；`/llm/jsim/excute` 收到 `cmd == "RUN_UAV_UID"` 且 UID 非空时，仅对应 UID 进入 `working`。状态登记不依赖后续控制流程是否接受或执行指令。
- 首次收到有效 `/llm/jsim/status/task_info` 时，只记录 `TM` 基准，不重置已激活装备。
- 后续有效 `TM` 大于基准时，全部装备恢复 `idle`，包括不在当前 `platform_list` 中的装备；相同或更旧的 `TM` 不改变状态，也不回退基准。
- 有效状态重置报文须包含数组 `CTN.platform_list`，且 `TM` 为正整数毫秒时间戳（JSON 数字，最大为 9007199254740991）。无效报文不更新状态基准。
- 状态变化在下一次位置上报时体现，不额外发布无位置依据的 `QB/303`；位置变化、静止时长和高度均不参与状态判断。
- 不新增任务类型缓存、定时器或线程。本地 `planZB()` 仿真播放分支保持原有状态逻辑。
- 当前 `TM` 来自报文组装时间，不是方案版本或完成通知；相同方案重新上报，只要 `TM` 增大也会重置全部装备状态。

### 6.2 任务完成通知

**代码位置**：`Src/xjsim/xjsim.cpp:1617-1622`

订阅 `llm/task/zc/command`，收到 `data == "dj_finish"` 时清除路线标绘。

---

## 7. 关键数据结构

### 7.1 MQTTMission::Position

```cpp
// Src/gkd/mission.h:52-68
class Position {
public:
    double longitude;  // 经度
    double latitude;   // 纬度
    double altitude;   // 高度
    double speed;      // 速度
    QJsonArray action_list; // 航点动作列表
};
```

### 7.2 excuteUAVOrUGV

```cpp
// Src/xjsim/xjsim.h:30-43
struct excuteUAVOrUGV {
    string uid;              // 装备编号
    string uname;            // 装备名称
    string cmd;              // "RUN_UAV_UID" 或 "RUN_UGV_UID"
    string task_type;        // "1"=侦查, "2"=打击
    int platform_type;       // 平台类型
    double executeHeight;    // 执行高度
    double waypointSpeed;    // 航点速度
    double roll, pitch, yaw; // 姿态角 (optional)
    std::vector<Position> path; // 路径点序列
};
```

### 7.3 TPCallback

```cpp
// Src/gkd/task_response_define.h:105
typedef std::function<void(string_view& root)> TPCallback;
```

RPC 回调类型，使用 `string_view` 实现零拷贝参数传递。

---

## 8. 配置文件参考

### 8.1 gkd_config.ini（关键节选）

```ini
[MQTT]
ip=192.168.3.101
port=18830

[RPC]
ip=192.168.1.65
port=9000

[GKD_VIDEO]
rtmpserver_ip=192.168.43.220

[ZYC]
1002_1=dy99
1002_2=dy100
1002_3=dy101
1002_4=dj01
1002_5=dj02
1002_6=dj03
2003_1=dy20
2003_2=bd01
```

### 8.2 config.ini（RPC 连接配置）

```ini
[RPC]
ip=192.168.1.65
port=9000
```

此配置用于 `rest_rcp_client` 连接到国遥 3D 引擎的 RPC Server。

---

## 9. 新增无人机装备步骤

假设需要新增一架编号为 `dj04` 的大疆 Mini 4 Pro：

### Step 1：修改 gkd_config.ini

在 `[ZYC]` 节添加：
```ini
1002_7=dj04
```

### Step 2：确认 RTMP 视频流

确保视频服务器 `192.168.43.220` 上有对应的 RTMP 流：
```
rtmp://192.168.43.220:1935/live/dj04
```

### Step 3：（可选）添加中文名映射

如果需要在位置上报时有中文名，修改 `Src/xjsim/xjsim.cpp:1589-1592`：
```cpp
if(uid == "dj04") uname = "无人机4";
```

### Step 4：无需代码改动（如果仅使用 RPC dotask 路径）

通过国遥引擎 `dotask` 下发任务时，`model_id` 填写 `dj04`，系统会自动：
1. `hasId(zyc["1002"], "dj04")` → `true`
2. 调用 `ControlDJiMini4_Pro("dj04", ...)` → 发布 `ZK/PathTopic`

### Step 5：如果使用 MQTT /llm/jsim/excute 路径

同样只需在 `[ZYC]` 中注册即可，MQTT 消息中 `uid` 填写 `dj04`。

**总结：新增装备的核心操作就是在 `[ZYC]` 中添加一行 `1002_N=装备编号`，代码零改动。**

---

## 10. 常见问题排查

### Q1: 任务下发后 ZK/PathTopic 无消息

**排查步骤**：

1. **检查 [ZYC] 配置**：装备编号是否与任务中的 `model_id` 完全一致（包括大小写）
   - 常见错误：`1002_5=di02`（应该是 `dj02`）
2. **检查 MQTT Broker 连接**：`gkd_config.ini` 中 `[MQTT]` 的 ip/port 是否正确
3. **检查代码分支**：确认装备走的是 `zyc["1002"]` 分支而非 `zyc["2003"]`

### Q2: 视频流无法加载

1. 检查 RTMP 服务器是否正常运行：`rtmpserver_ip` 配置
2. 检查 `GKD_VIDEO` 节是否存在
3. 确认国遥引擎调用了 `videoStreamCall` RPC 方法

### Q3: 无人机位置不更新

1. 确认飞控在发布 `/aircraft/location` 主题
2. 检查消息格式：CSV `uid,lon,lat,alt`
3. 检查 MQTT Broker 是否连通

### Q4: 高度校验失败（RPC dotask 路径）

c2manager.cpp 中有严格的高度校验：
- 巡检高度 < 20m → 拒绝
- 多个无人机之间高度差 < 10m → 拒绝

如需要放宽限制，修改 `Src/gkd/c2manager.cpp:392-398` 的判断逻辑。

---

## 附录：关键源文件索引

| 文件 | 职责 |
|------|------|
| `Src/common/gkd_config.h` | 配置结构体、ZYC 资源池加载 |
| `Src/common/common_config.h` | RPC ip/port 配置 |
| `Src/gkd/c2manager.cpp` | RPC 回调处理（dotask/videoStreamCall）、ZK/PathTopic 发布 |
| `Src/gkd/c2manager.h` | C2Manager 类声明 |
| `Src/xjsim/xjsim.cpp` | MQTT 回调处理、/aircraft/location 订阅、ZK/PathTopic 发布 |
| `Src/xjsim/xjsim.h` | XjSim 类声明、excuteUAVOrUGV 结构体 |
| `Src/gkd/task_response_define.h` | TPCallback 类型定义 |
| `Src/gkd/mission.h` | MQTTMission::Position 等数据结构 |
| `gkd_config.ini` | 运行时配置（MQTT/RPC/ZYC/Video） |
| `config.ini` | RPC 连接配置 |
