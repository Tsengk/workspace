# 云台 MQTT 接口文档

| 项目 | 内容 |
| :--- | :--- |
| 文档版本 | V1.0 |
| 编写日期 | 2026-11-15 |
| 实现基线 | 独立云台 MQTT 通道实现 |
| 适用功能 | 远程控制云台 Pitch 轴，接收执行回执和姿态快照 |
| 读者 | 控制端开发、Android 开发、联调和测试人员 |


## 1. 接口总览

| Topic | 方向 | 发布方 | 订阅方 | 用途 |
| :--- | :--- | :--- | :--- | :--- |
| `ZK/GimbalTopic` | 下行 | 控制端 | Android 云台模块 | 下发 Pitch 角度命令 |
| `ZK/GimbalReplyTopic` | 上行 | Android 云台模块 | 控制端 | 接收确认、执行成功或失败回执 |
| `ZK/GimbalStateTopic` | 上行 | Android 云台模块 | 控制端 | 执行结果携带的连接及姿态快照 |

三个 Topic 大小写敏感，不带前导 `/`

### 1.1 通信约定

| 项目 | 约定 |
| :--- | :--- |
| 协议 | 设备端使用 MQTT 5 |
| Broker | 与 Android 配置 `mqttip`、`mqttport` 一致；设备端未配置 TLS、用户名或密码 |
| 消息编码 | UTF-8 JSON；每条 MQTT 消息为一个 JSON 对象 |
| 命令 QoS | 设备订阅请求 QoS 2；控制端建议使用 QoS 1 或 2，实际接收等级受双方及 Broker 协商限制 |
| 回执和状态 QoS | 设备发布请求 QoS 2，实际支持能力由 Broker 决定 |
| Retain | 命令必须为 false；设备回执、状态均以 false 发布 |
| 时间单位 | Unix 毫秒时间戳；不要传 Unix 秒数，也不要加减时区偏移 |
| 角度单位 | 度（°） |
| 空值输出 | Gson 默认省略 null 字段，不保证字段顺序 |
| 关联键 | 回执用 `rid + command_id`；状态只有 rid 和时间戳，没有 command_id |

接入前应先订阅两个上行 Topic，再发送命令。云台模块当前由航线页面初始化启动。

### 1.2 路由与设备过滤

- 仅 `ZK/GimbalTopic` 上的消息进入云台业务，其他 Topic 不执行云台动作。
- 基础解析成功后，将消息 rid 去首尾空白，与 Android 配置的 rid 比较；不匹配则静默忽略，不返回 TARGET_MISMATCH。
- retained 命令被忽略，不执行动作，也不返回业务失败回执。
- **先解析、后匹配 rid**：解析失败的消息可能由多台设备分别返回失败回执，回执 rid 表示实际响应设备。

## 2. 下行：ZK/GimbalTopic

### 2.1 功能说明

当前仅支持 `gimbal_pitch`，执行一次相对或绝对角度旋转。只控制 Pitch，SDK 调用显式忽略 Roll/Yaw。不支持通过此接口查询状态、控制 Roll/Yaw 或持续速度旋转。

### 2.2 请求字段

字段名必须准确，使用小写下划线形式。

| 字段 | JSON 类型 | 必填 | 默认值 / 约束 |
| :--- | :--- | :--- | :--- |
| `type` | string | 是 | 固定为 `gimbal_pitch`，大小写敏感 |
| `rid` | string | 是 | 非空且非纯空白；与设备配置匹配 |
| `command_id` | string | 是 | 非空且非纯空白；控制端应生成唯一值；代码不限制具体格式 |
| `mode` | string | 是 | `relative` / `absolute`，不区分大小写，不自动去首尾空白 |
| `pitch` | number | 是 | 有限数值，不能是 null、NaN 或无穷；范围见下节 |
| `gimbal_index` | string | 否 | 省略、null、空字符串或纯空白均默认 `LEFT_OR_MAIN`；支持 `LEFT_OR_MAIN`、`RIGHT`、`UP`，不区分大小写并去首尾空白 |
| `timestamp` | integer | 否 | 省略或 null 时跳过过期检查；传入时必须为正的 Unix 毫秒整数 |

注意事项：

- `command_id` 不做 trim，应避免前后空格，每个新动作使用新 ID。
- 未知字段会被忽略，不会自动提示拼写错误。例如 `gimbsl_index` 不生效，正确字段为 `gimbal_index`；拼错后会退回默认云台。
- 建议严格使用表中类型，不依赖 Gson 对字符串/数字的隐式转换。
- 不要用 `timestamp: 0` 或空字符串表示不提供时间，直接省略字段。

### 2.3 角度与执行限制

| 条件 | 当前行为 |
| :--- | :--- |
| relative | pitch 表示在当前角度基础上的增量，范围 `[-30, 30]`，包含端点 |
| absolute | pitch 表示目标角度，优先使用设备上报的 Pitch 范围 |
| 设备范围不可用或无效 | absolute 回退范围 `[-90, 30]`，包含端点 |
| 超出允许范围 | 拒绝执行，返回 INVALID_PITCH_RANGE；不是自动截断 |
| 航线状态为 EXECUTING | 默认拒绝，返回 MISSION_CONFLICT |
| 航线暂停或状态未知 | 不被当前 EXECUTING 判断直接禁止，仍需通过其他校验 |
| 云台校准中 | 返回 GIMBAL_BUSY |
| SDK旋转参数 | duration=1.0 秒；不代表承诺1秒内收到最终回执 |
| SDK等待超时 | 默认10秒；超时回执不等于硬件动作已取消 |

正 Pitch / 负 Pitch 直接传给 SDK，不在此层取反；“正向上、负向下”的物理方向需按实际机型联调确认。若绝对目标与当前角度相同，成功也可能看不到明显动作。

### 2.4 时间戳规则

执行器计算：

```text
age = Android设备当前Unix毫秒时间 - 请求timestamp
age > 10000 → COMMAND_TIMEOUT / command expired
```

- 恰好10,000毫秒不因这一条件拒绝；实际网络延迟和调度会影响检查时刻。
- 当前未单独拒绝未来时间戳，不能将此规则视为完整的时钟偏差或防重放校验。
- 请求时间来自控制端；回执和状态时间来自 Android，双方系统时间应同步。
- 省略 timestamp 可以执行，但失去过期命令过滤，建议只在联调时使用，正式控制发送时动态生成。
- `command_id` 中的日期只是字符串，不参与时效判断。

### 2.5 请求示例

#### 示例A：绝对旋转到−30°，不带时间戳

可用于临时排除时钟问题；替换为实际 rid，每次新动作更换 command_id。

```json
{
  "type": "gimbal_pitch",
  "rid": "zs02",
  "command_id": "gimbal-zs02-absolute-000001",
  "mode": "absolute",
  "pitch": -30,
  "gimbal_index": "LEFT_OR_MAIN"
}
```

#### 示例B：相对旋转−10°，使用默认云台

```json
{
  "type": "gimbal_pitch",
  "rid": "zs02",
  "command_id": "gimbal-zs02-relative-000002",
  "mode": "relative",
  "pitch": -10
}
```

## 3. 上行：ZK/GimbalReplyTopic

### 3.1 字段定义

| 字段 | JSON 类型 | 出现条件 / 含义 |
| :--- | :--- | :--- |
| `type` | string | 始终为 `gimbal_pitch`，包括未知type导致的错误回执 |
| `rid` | string | 响应设备的本机标识 |
| `command_id` | string | 可用时提供；解析或基础校验失败时省略，即使原始报文中带有ID |
| `status` | string | `accepted` / `success` / `failed` |
| `request_pitch` | number | success 中来自执行结果；相对模式时是增量，不是最终角度；failed 不包含 |
| `current_pitch` | number | success 时当前可读取的Pitch快照；不可用则省略 |
| `current_roll` | number | success 时当前可读取的Roll快照；不可用则省略 |
| `current_yaw` | number | success 时当前可读取的Yaw快照；不可用则省略 |
| `error_code` | string | failed 时提供；重复历史成功命令的success也会提供DUPLICATE_COMMAND |
| `error_message` | string | 可用时提供，人类可读诊断；不要依赖完整文本固定不变 |
| `timestamp` | integer | 本条回执生成时的设备Unix毫秒时间，不是Broker确认时间 |

### 3.2 状态语义

| status | 含义 | 不代表什么 |
| :--- | :--- | :--- |
| accepted | 基础解析成功、目标为本机，准备进入执行器校验 | 不表示时效、航线冲突、范围或设备能力已检查通过，也不表示动作成功 |
| success | SDK返回成功，或对已成功命令返回历史结果 | 不保证角度已稳定，不代表一定有可读姿态，不保证这是新的硬件动作 |
| failed | 基础校验、执行前检查、SDK动作或等待失败 | 不保证一定调用过SDK，也不保证动作已取消 |

正常命令通常是 `accepted → success/failed`。解析失败直接 failed，没有 accepted。每个回执都是独立MQTT消息，不能将两个JSON对象拼成一个JSON解析。网络中断、队列溢出等情况下不能保证两条都送达。

### 3.3 回执示例


#### 已接收

```json
{
  "type": "gimbal_pitch",
  "rid": "zs02",
  "command_id": "gimbal-zs02-absolute-000001",
  "status": "accepted",
  "timestamp": 1794710358479
}
```

#### SDK执行成功

```json
{
  "type": "gimbal_pitch",
  "rid": "zs02",
  "command_id": "gimbal-zs02-absolute-000001",
  "status": "success",
  "request_pitch": -30.0,
  "current_pitch": -30.0,
  "current_roll": 0.0,
  "current_yaw": 0.0,
  "timestamp": 1794710359480
}
```

#### 命令已过期，尚未调用旋转动作

```json
{
  "type": "gimbal_pitch",
  "rid": "zs02",
  "command_id": "gimbal-20261114-000001",
  "status": "failed",
  "error_code": "COMMAND_TIMEOUT",
  "error_message": "command expired (age=60358479ms)",
  "timestamp": 1794710358480
}
```
表示发送控制的时间晚与app端的时间为10s导致控制超时，在控制端可以去掉时间字段跳过校验。

#### 基础字段非法：没有command_id关联字段

```json
{
  "type": "gimbal_pitch",
  "rid": "zs02",
  "status": "failed",
  "error_code": "INVALID_COMMAND",
  "error_message": "mode must be relative or absolute",
  "timestamp": 1794710358480
}
```

JSON中的 `\u003d` 与 `=` 等价，是序列化转义，不是协议错误；应先用JSON解析器解码。

### 3.4 错误码

| error_code | 原因 | 是否可能伴随状态快照 |
| :--- | :--- | :--- |
| INVALID_JSON | 空payload、反序列化异常或无法解析为命令 | 否 |
| INVALID_COMMAND | 不支持type、缺rid/command_id、mode非法、Pitch缺失/非有限数、timestamp≤0 | 否 |
| TARGET_MISMATCH | 执行器保留的目标不匹配保护 | 正常MQTT入口已静默过滤，通常不产生此回执 |
| DUPLICATE_COMMAND | 同command_id正在执行，或返回其历史完成结果 | 执行中否；历史结果视原缓存是否带state |
| GIMBAL_NOT_CONNECTED | 无法确认云台连接或读取状态失败 | 是，可为connected=false |
| GIMBAL_NOT_SUPPORTED | 索引非法，或旋转Key不受支持 | 非法索引否；能力不支持时是 |
| INVALID_PITCH_RANGE | Pitch超出相对限幅或绝对范围 | 否 |
| GIMBAL_BUSY | 云台校准中；不是所有并发动作的统一忙判断 | 是 |
| MISSION_CONFLICT | 航线EXECUTING且默认不允许控制 | 否 |
| SDK_ACTION_FAILED | SDK失败回调或动作调用抛异常 | 是 |
| COMMAND_TIMEOUT | 命令年龄超过10秒，或下发SDK后等待超过10秒 | 过期否；SDK等待超时是 |

“可能伴随状态”表示业务结果带state，仍受发布队列和网络影响，不保证状态消息必达。多个条件同时不满足时，按代码校验顺序返回先遇到的错误，例如航线冲突检查早于过期检查。

### 3.5 幂等和重试

- 幂等键仅为command_id，不包含模式、角度或云台索引。同ID改参数不是新动作，不应这样使用。
- 执行中重复：返回failed / DUPLICATE_COMMAND，不重复下发SDK。
- 已完成重复：沿用缓存的success/failed，并附DUPLICATE_COMMAND；历史姿态可能已过时。
- 已完成成功的重复回执可以同时存在 `status=success` 和 `error_code=DUPLICATE_COMMAND`，控制端必须兼容。
- 只缓存当前执行器最近256条完成结果；重建管理器或缓存淘汰后可能再次执行旧ID，不是永久幂等。
- 执行前被拒绝的命令（例如过期、任务冲突、越界）不写入完成结果缓存，后续同ID可能重新进入校验。
- 控制端应区分“重试同一逻辑命令”和“发起新动作”；丢失最终回执后不要盲目换新ID重复发送相对动作，以免重复旋转。

## 4. 上行：ZK/GimbalStateTopic

### 4.1 字段定义

| 字段 | JSON 类型 | 必有 | 含义 |
| :--- | :--- | :--- | :--- |
| `rid` | string | 是 | 设备标识 |
| `connected` | boolean | 是 | 读取的目标云台连接状态；读取异常时回退false，不是MQTT连接状态 |
| `pitch` | number | 否 | SDK可读取的Pitch，单位度 |
| `roll` | number | 否 | SDK可读取的Roll，单位度 |
| `yaw` | number | 否 | SDK可读取的Yaw，单位度 |
| `timestamp` | integer | 是 | 状态对象生成时的设备Unix毫秒时间 |

状态没有 `type`、`status`、`command_id`、`gimbal_index`。多云台情况下，当前协议无法从状态报文本身可靠识别云台索引；并发命令也不能靠“最近一条状态”精确关联，控制端应以回执为主要执行依据。

### 4.2 触发条件

状态为执行结果驱动，不是定时上报：

- SDK成功、失败或等待超时后读取快照，结果带state时发布。
- 云台未连接、能力不支持、校准中等部分分支也会带state并发布。
- 解析失败、非本机、retained、命令过期、任务冲突、角度越界等早期分支不产生状态。
- 不在MQTT连接成功时自动发布，不在每条accepted后保证发布，不在云台姿态变化时自动推送。
- 历史重复命令可能重新发布缓存旧状态，其timestamp仍是原快照时间。

因此，**没有状态消息不能直接说明状态通道异常；应先查看同一命令的最终回执。**

### 4.3 消息示例

可读取姿态：

```json
{
  "rid": "zs02",
  "connected": true,
  "pitch": -30.0,
  "roll": 0.0,
  "yaw": 0.0,
  "timestamp": 1794710359479
}
```

未连接或读取异常，角度字段不可用时省略：

```json
{
  "rid": "zs02",
  "connected": false,
  "timestamp": 1794710359480
}
```

## 5. 时序与无响应场景

### 5.1 正常时序

```text
控制端                        Android云台模块                     DJI SDK
  |  订阅Reply和State              |                                |
  |-- GimbalTopic:命令 ----------->|                                |
  |                               | Topic/retain/解析/rid检查         |
  |<-- Reply:accepted ------------|                                |
  |                               | 时效/航线/范围/连接/能力检查       |
  |                               |-- RotateByAngle -------------->|
  |                               |<-- 成功或失败回调 ---------------|
  |<-- Reply:success/failed -------|                                |
  |<-- State:状态快照（如有） ------|                                |
```

图中箭头表示逻辑提交顺序，不是端到端交付保证。设备对同一命令先将accepted入队，再提交最终回执、状态；跨Topic关联、并发命令和网络重传不能靠到达顺序替代command_id。

### 5.2 没有回执的可能原因

- 未进入对应页面、管理器未启动、功能开关关闭或配置无效。
- 发错Topic、合法命令rid不匹配、命令Retain开启。
- 连接/订阅未完成、Broker不可达或ACL不允许。
- 发布队列已满、消息本地等待超过10秒或连接中断；accepted入队失败时不会继续执行该命令。
- SDK迟到回调来自已停止的业务会话或旧连接，按隔离策略丢弃，不跨重连补发。

设备发布队列上限128条；断线清空，不持久化。重连以1秒递增退避到最多30秒，不缓存或主动重放离线运动命令。MQTT QoS不替代应用层的执行确认和持久幂等。
