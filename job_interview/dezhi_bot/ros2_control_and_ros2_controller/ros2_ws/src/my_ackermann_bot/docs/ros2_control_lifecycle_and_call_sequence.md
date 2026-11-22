# ros2_control 生命周期与函数调用时序

本文按当前工程的 **ROS 2 Foxy API** 梳理 `my_ackermann_bot` 中控制器、Controller Manager、ResourceManager 和硬件插件的完整关系。重点追踪接口如何暴露、申请、更新，以及状态和命令如何进出硬件。

> Foxy 这套实现中没有一个额外的 `HardwareManager` 对象负责搬运接口值。`ResourceManager` 维护硬件组件和接口资源；`ControllerManager` 管理控制器、接口分配和周期调度；具体硬件动作由 `hardware_interface::SystemInterface` 插件实现。

## 1. 总览

```text
操作者 / 导航节点
   │  /ackermann_controller/cmd_vel (Twist)
   ▼
AckermannController ROS 订阅回调
   │  RealtimeBuffer
   ▼
ControllerManager 控制周期
   ├── ① ResourceManager::read()
   │       └── SystemInterface::read()：刷新 state 数值
   ├── ② AckermannController::update()
   │       └── 写入 loaned command interfaces
   └── ③ ResourceManager::write()
           └── SystemInterface::write()：消费 command 数值
                   ├── GazeboSystem → Gazebo 关节
                   └── AckermannCanopenSystem → 内部缓存 → 后台 ROS 客户端
                                                     → MotionControllerDriver
                                                     → CANopen → 电机
```

接口路径可简写为：

```text
硬件状态 → read() → StateInterface → Controller
Controller → CommandInterface → write() → 硬件命令
```

`CommandInterface` 和 `StateInterface` 是带名称的数值句柄，不是 ROS Topic。当前自定义硬件插件将句柄绑定到自身 `double` 数组元素的地址；控制器与硬件插件访问同一个数值，不经过 ResourceManager 的逐周期复制。

## 2. 组件职责

| 组件 | 当前工程中的职责 |
|---|---|
| `AckermannController` | 订阅 `Twist`，计算转向角和驱动轮角速度，将结果写到命令接口。它不操作 CAN。 |
| `ControllerManager` | 通过服务加载、配置、激活和停用控制器；在控制周期中调用 read、update、write。 |
| `ResourceManager` | 从 URDF 加载硬件插件，登记 command/state interfaces，按接口名称借给控制器，并将 read/write 分派给硬件组件。 |
| `AckermannCanopenSystem` | 适配 ros2_control 关节接口到 ros2_canopen 服务和反馈话题。 |
| `MotionControllerDriver` | 提供 CANopen 电机目标、状态读取、halt 和反馈等驱动层功能。 |
| `joint_state_broadcaster` | 读取 state interfaces 并发布关节状态；不写电机命令。 |

## 3. 当前暴露的接口清单

### 3.1 ROS 话题和服务

| 名称 | 类型 | 方向 / 用途 |
|---|---|---|
| `/ackermann_controller/cmd_vel` | `geometry_msgs/msg/Twist` | 输入；控制器使用 `linear.x`（m/s）和 `angular.z`（rad/s）。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出；由 `joint_state_broadcaster` 根据硬件 state interfaces 发布。 |
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | 查询控制器状态。 |
| `/controller_manager/list_hardware_interfaces` | `controller_manager_msgs/srv/ListHardwareInterfaces` | 查询已注册及已 claim 的硬件接口。 |
| `/controller_manager/load_controller`、`configure_controller`、`switch_controller` 等 | `controller_manager_msgs` 服务 | 管理控制器插件和生命周期。 |
| `/ackermann_canopen_hardware/arm` | `std_srvs/srv/Trigger` | CANopen 桥接层显式解锁；不是激活控制器。 |
| `/ackermann_canopen_hardware/disarm` | `std_srvs/srv/Trigger` | 请求对各电机发送 halt；响应成功不代表物理停车已确认。 |
| `/ackermann_canopen_hardware/armed` | `std_msgs/msg/Bool` | 输出桥接层的软件 armed 状态。 |
| `/ackermann_canopen_hardware/status` | `std_msgs/msg/String` | 输出桥接层状态/故障描述。 |

CANopen 硬件插件还连接每个 `driver_node` 下的：

| 每电机接口 | 类型 | 用途 |
|---|---|---|
| `/<driver_node>/target` | `canopen_interfaces/srv/COTargetDouble` | 发送单轴原始目标。 |
| `/<driver_node>/sdo_read` | `canopen_interfaces/srv/CORead` | 读取 `0x6041` 状态字、`0x6061` 模式显示。 |
| `/<driver_node>/halt` | `std_srvs/srv/Trigger` | 请求驱动器 halt。 |
| `/<driver_node>/actual_position` | `std_msgs/msg/Float64` | 电机原始位置反馈。 |
| `/<driver_node>/actual_speed` | `std_msgs/msg/Float64` | 电机原始速度反馈。 |

这些电机接口属于当前 CANopen 后端适配，不是 ros2_control 的标准接口。Gazebo 模式使用 GazeboSystem，不使用这组 CANopen 电机接口。

### 3.2 ros2_control command/state interfaces

CANopen Xacro 为四个关节各声明一个命令接口和 position、velocity 两个状态接口，见 `my_ackermann_bot/urdf/ros2_control_canopen.xacro`。当前硬件导出：

| 关节 | Command interface | State interfaces |
|---|---|---|
| `front_left_steer_joint` | `position`（rad） | `position`（rad）、`velocity`（rad/s） |
| `front_right_steer_joint` | `position`（rad） | `position`（rad）、`velocity`（rad/s） |
| `rear_left_wheel_joint` | `velocity`（rad/s） | `position`（rad）、`velocity`（rad/s） |
| `rear_right_wheel_joint` | `velocity`（rad/s） | `position`（rad）、`velocity`（rad/s） |

接口完整名称由 `<joint name>` 和 `<interface name>` 组成，例如：

```text
front_left_steer_joint/position
rear_left_wheel_joint/velocity
```

Command 和 state 属于两类独立注册资源；即使关节和接口字符串相同，也分别在命令、状态集合中管理。

当前 `AckermannController::command_interface_configuration()` 申请四个命令接口：两个前轮 position、两个后轮 velocity。它的 `state_interface_configuration()` 申请四个状态接口：两个前轮 position、两个后轮 velocity。它**不**申请后轮 position 或前轮 velocity；其他消费者（如 broadcaster）可读取剩余 state interfaces。

Gazebo Xacro 目前为前转向关节只声明 position state、为后轮声明 position 和 velocity state，见 `my_ackermann_bot/urdf/ros2_control.xacro`。控制器的 state 请求与 Gazebo 声明相匹配。

### 3.3 硬件插件 API 与控制器 API

`SystemInterface` 插件实现的主要 API：

```text
configure(info)                 接收解析后的 ros2_control 硬件配置
export_state_interfaces()      暴露状态接口句柄
export_command_interfaces()    暴露命令接口句柄
start()                        启动硬件数据交换
read()                         刷新状态数据
write()                        消费命令数据
stop()                         停止硬件数据交换
```

Controller 插件实现的主要 API：

```text
init(name)                            创建/初始化控制器实例并声明参数
command_interface_configuration()     声明需要的 command interfaces
state_interface_configuration()        声明需要的 state interfaces
on_configure()                         读取参数、创建订阅、检查配置
on_activate()                          开始控制前初始化目标
update()                               每个控制周期运行
on_deactivate()                        停止控制器命令输入并进入非 active 状态
```

## 4. 启动与硬件初始化时序

当前统一入口是 `my_ackermann_bot/launch/ackermann.launch.py`。Gazebo 模式 include 原 Gazebo launch；`mock` 和 `canopen` 模式启动 robot_state_publisher、独立 `ros2_control_node` 和控制器启动脚本。

### 阶段 A：启动描述和 ResourceManager 建立

```text
ros2 launch
  ↓
launch_setup()
  ↓
处理 mode、YAML、Xacro 参数
  ↓
xacro.process_file() → robot_description XML
  ↓
ros2_control_node
  ↓
ControllerManager
  ↓
ResourceManager 从 robot_description 加载 <ros2_control>
  ↓
pluginlib 创建指定的 SystemInterface 插件
  ↓
硬件插件 configure(HardwareInfo)
  ↓
export_state_interfaces()
  ↓
export_command_interfaces()
  ↓
ResourceManager 建立接口名称 → 接口句柄的注册表
  ↓
硬件插件 start()
```

CANopen 的 `configure()` 解析每个 joint 的驱动节点名、单位比例、方向、限制和模式等参数。`export_*_interfaces()` 将接口绑定到插件内部数组。`start()` 创建适配节点、订阅反馈、创建电机服务客户端，启动后台 executor，并等待各轴初始反馈/状态满足启动条件；等待时间由 `startup_timeout_ms` 配置。

概念上的内存映射如下：

```text
StateInterface("front_left_steer_joint", "position", &positions[0])
StateInterface("front_left_steer_joint", "velocity", &velocities[0])
CommandInterface("front_left_steer_joint", "position", &commands[0])
```

对应实现见 `ackermann_canopen_hardware/src/ackermann_canopen_system.cpp` 的 `export_state_interfaces()` 和 `export_command_interfaces()`。

### 阶段 B：Controller 插件加载和配置

```text
spawn_ackermann_controllers.py
  ↓ 等待 /controller_manager/list_controllers 可响应
  ↓
spawner.py joint_state_broadcaster
  ↓ pluginlib 创建 broadcaster，ControllerManager 调 init()
  ↓ configure()
  ↓ 申请/claim 需要的 state interfaces

spawner.py ackermann_controller
  ↓
pluginlib 按 YAML 类型创建 my_ackermann_bot::AckermannController
  ↓ init("ackermann_controller")
  ↓ on_configure()
  ↓ Controller 状态进入 inactive
```

`init()` 声明可配置参数；`on_configure()` 读取 `ackermann_controllers.yaml` 或 `ackermann_controllers_real.yaml` 中的关节名、轴距、轮距、轮径和限幅，校验数值，并创建私有话题 `~/cmd_vel` 订阅。实例名是 `ackermann_controller`，所以该私有话题解析为 `/ackermann_controller/cmd_vel`。

此时控制器已经加载和配置，但尚未运行周期 `update()`。

### 阶段 C：激活控制器和接口借用

激活时 ControllerManager 大致执行：

```text
读取 command_interface_configuration()
读取 state_interface_configuration()
  ↓
ResourceManager 按名称查找接口
  ↓
claim command interfaces（命令接口独占）
claim state interfaces（状态接口只读，可多方读取）
  ↓
生成 LoanedCommandInterface / LoanedStateInterface
  ↓
ControllerInterface::assign_interfaces(...)
  ↓
AckermannController::on_activate()
  ↓
Controller 状态进入 active
```

`AckermannController::on_activate()` 检查拿到的接口数量，以当前前轮 state position 初始化转向 command position，并把后轮速度 command 设为零；它同时清空旧指令缓存、开启新指令接收。这样重新激活时不会直接把转向目标重置到零。

命令接口有唯一写入控制器；尝试激活另一个同时申请同一命令接口的控制器会发生 resource conflict。Loaned interface 生命周期结束（控制器停用/释放）时，命令资源 claim 会被归还。

## 5. 每个控制周期的调用和数据时序

实机/mode mock 的 `controller_manager` 配置为 `update_rate: 100`，目标周期约 10 ms。Gazebo 模式由 Gazebo 插件持有 ControllerManager，其更新由仿真循环驱动。默认周期的逻辑顺序如下：

```text
ControllerManager 控制循环
  │
  ├─ 1. ControllerManager::read()
  │      └─ ResourceManager::read()
  │             └─ AckermannCanopenSystem::read()
  │                    └─ feedback → positions / velocities
  │
  ├─ 2. ControllerManager::update()
  │      └─ 遍历 active controllers
  │             └─ AckermannController::update()
  │                    ├─ RealtimeBuffer::readFromRT()
  │                    ├─ 检查命令有效性、超时、速度限幅
  │                    ├─ 计算两个转向角和两个车轮角速度
  │                    └─ LoanedCommandInterface::set_value()
  │                           └─ 修改硬件插件 commands 数组元素
  │
  └─ 3. ControllerManager::write()
         └─ ResourceManager::write()
                └─ AckermannCanopenSystem::write()
                       ├─ 检查 NaN 和关节限值
                       └─ commands → latest_commands 缓存
```

### 命令输入与实时缓冲

ROS 订阅回调不直接调用控制器 `update()`，也不直接写硬件接口。回调只校验 `linear.x`、`angular.z`，记录 `steady_clock` 时间，并把命令写入 `RealtimeBuffer`：

```text
Twist 消息
  → subscription callback（非实时上下文）
  → RealtimeBuffer::writeFromNonRT()
  → 控制循环的 update() 读取
```

控制器 `update()` 执行前进/倒车/转弯几何运算，再写四个 command interfaces。当前转向命令保存在 command interface 中；`get_value()` 读取的是上次的**命令目标**，并不等价于传感器的实际位置。实际位置要从 state interface 读。

### `set_value()` 如何更新命令

```text
command_interfaces_[i].set_value(value)
  → LoanedCommandInterface
  → CommandInterface
  → 对应 value_ptr 指向的硬件插件变量
  → impl_->commands[i] = value
```

所以 ResourceManager 不会每周期将 command 数组再复制、转发给另一个 manager；接口句柄关联同一内存位置。ResourceManager 的职责是登记、查找、管理 claim，并在周期中调用硬件 `write()`。

### 状态反馈如何更新

CANopen 反馈不是在 ros2_control `read()` 中同步等待 ROS 服务，而是由硬件插件自己的 ROS 订阅回调持续接收：

```text
MotionControllerDriver 发布 actual_position / actual_speed
  → AckermannCanopenSystem 订阅回调
  → 更新 feedback 缓存（原始电机单位转为 rad / rad/s）
  → 周期 SystemInterface::read()
  → positions / velocities 数组
  → StateInterface 指针所指的数值
  → Controller 的 LoanedStateInterface::get_value()
```

当前 `read()` 用 try-lock 拷贝反馈快照，避免控制周期等待后台 ROS 回调持有的互斥锁。反馈路径代码见 `ackermann_canopen_system.cpp` 中的订阅回调和 `read()`。

## 6. write 后的 CANopen 后台链路

CANopen 硬件插件的 ros2_control `write()` 不是同步发送/等待电机服务；它校验命令后，将 `commands` 快照写入 `latest_commands` 并更新时间戳。后台 wall timer 随后处理：

```text
AckermannCanopenSystem::write()
  → latest_commands
  → 后台 timer tick()
  → 检查 arm、控制指令新鲜度、反馈新鲜度、CiA402 状态/模式
  → 速度/加速度限制
  → rad / rad/s 到电机原生单位换算
  → async_send_request(<motor>/target)
  → MotionControllerDriver
  → Lely / CANopen
  → 电机驱动器
```

后台还通过 `sdo_read` 轮询 `0x6041`（状态字）和 `0x6061`（模式显示），目标请求最多一个在途请求/轴。ROS 服务完成回调会检查驱动响应；异常和超时锁存故障并请求所有轴 halt。该实现是服务桥接，不保证四轴 PDO 同步或确定性实时。

## 7. Controller 生命周期、接口生命周期、硬件生命周期的区别

```text
Controller：
unconfigured --configure/on_configure--> inactive
inactive --claim interfaces + activate/on_activate--> active
active --deactivate/on_deactivate + release loans--> inactive
inactive --cleanup--> unconfigured

Hardware SystemInterface（Foxy 接口）：
configure(info) → export interfaces → start()
                                  → read()/write() 周期调用
                                  → stop() 停止数据交换
```

控制器的 lifecycle 和硬件插件的启动/停止**不是同一个状态机**。尤其：

- 停止 `ackermann_controller` 只会停用控制器并释放它借用的接口；通常不会因此调用 `AckermannCanopenSystem::stop()`。
- 控制器停用也不等于执行 `/ackermann_canopen_hardware/disarm`。当前 `on_deactivate()` 将后轮 command 置零、停止接收 Twist，但不会调用硬件插件的服务。
- 硬件组件仍运行时，ResourceManager 的硬件周期仍可继续读写；CANopen 后台 executor 也仍运行。
- CANopen `/disarm` 会由硬件适配节点向各轴请求 halt，并把 `armed` 清除；它不等价于卸载或停用控制器。控制器仍可计算、写 command interface，但后台不会在 disarmed 状态发送正常目标。
- `SystemInterface::stop()` 是停止硬件交换的生命周期动作，通常在资源管理器/进程关闭硬件时执行，不是日常 controller switch 的同义词。

实机操作时，应区分三个动作：

```text
停控制器：switch_controller --stop ackermann_controller
硬件 disarm：调用 /ackermann_canopen_hardware/disarm
停止硬件组件：ControllerManager/ResourceManager 关闭时调用 SystemInterface::stop()
```

不要将其中任一动作单独视为经过验证的紧急停车。急停/STO、驱动 watchdog 和制动策略必须独立设计与验证。

## 8. 切换控制器时序

启动控制器的服务调用（例如 `ros2 control switch_controllers --start ...`）由 ControllerManager 处理。概念流程：

```text
switch_controller(start/stop)
  → 检查待停止/启动控制器
  → 停止控制器：deactivate，归还已有 loaned command interfaces
  → 为待启动控制器查询接口需求
  → ResourceManager 检查命令接口是否可 claim
  → loan interfaces + assign_interfaces()
  → on_activate()
  → 更新 active controller 列表
  → 后续控制周期开始调用新控制器 update()
```

若硬件使用不同 command mode，ResourceManager 还提供 `prepare_command_mode_switch()` 和 `perform_command_mode_switch()` 钩子。当前阿克曼硬件固定为前轮 position、后轮 velocity，没有在此钩子中执行额外模式切换；CANopen 插件要求驱动器事先处于期望的 PP/PV 模式。

## 9. 停止和故障时序

### 正常关闭

```text
Ctrl-C / launch 关闭
  → 停止控制器并释放接口（具体 shutdown 顺序受进程关闭流程影响）
  → ResourceManager 停止硬件组件
  → AckermannCanopenSystem::stop()
  → Impl::shutdown()
  → 清除 armed、请求 halt
  → 取消 executor 并 join 后台线程
```

插件的关闭实现见 `ackermann_canopen_system.cpp` 中 `shutdown()`。halt 是尽力而为的通信请求，不是物理停止确认；掉线、主机崩溃或进程被强杀时，必须依赖独立安全回路。

### 控制指令超时

```text
cmd_vel 超过控制器 cmd_vel_timeout
  → AckermannController::update() 将车速/角速度视为 0
  → 后轮目标归零、转向目标保持
  → 后续 ros2_control write() 更新硬件命令缓存
  → CANopen 后台按加速度限制发送零速目标
```

CANopen 插件另有命令流超时、反馈超时、SDO 状态/模式错误、目标拒绝和服务超时检测。检测到硬件桥接故障后会锁存 fault、停止发新目标并请求各轴 halt；通常需要修正问题并重启 bringup 清除锁存故障。

## 10. 用当前源码定位各阶段

| 要理解的问题 | 源码入口 |
|---|---|
| Launch 如何选择 Gazebo / mock / CANopen、启动 node | `my_ackermann_bot/launch/ackermann.launch.py` |
| Controller Manager 的控制器类型与周期参数 | `my_ackermann_bot/config/ackermann_controllers.yaml`、`my_ackermann_bot/config/ackermann_controllers_real.yaml` |
| 模拟硬件暴露哪些关节接口 | `my_ackermann_bot/urdf/ros2_control.xacro` |
| CANopen 硬件暴露哪些关节接口和参数 | `my_ackermann_bot/urdf/ros2_control_canopen.xacro` |
| 控制器申请哪些接口 | `my_ackermann_bot/src/ackermann_controller.cpp` 中 `command_interface_configuration()` 和 `state_interface_configuration()` |
| 控制器生命周期和周期算法 | 同文件中的 `init()`、`on_configure()`、`on_activate()`、`update()`、`on_deactivate()` |
| 硬件将接口绑定到哪些变量 | `ackermann_canopen_hardware/src/ackermann_canopen_system.cpp` 中 `export_*_interfaces()` |
| 硬件如何读写 ros2_control 值 | 同文件中的 `read()` 和 `write()` |
| command 缓存如何变成电机服务请求 | 同文件中的 `tick()`、`poll_status()` 和 `target->async_send_request()` |
| Controller 插件如何被发现 | `my_ackermann_bot/ackermann_controller_plugin.xml`、`my_ackermann_bot/CMakeLists.txt` |

## 11. 运行时检查

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /ackermann_controller/cmd_vel
ros2 topic echo /joint_states
ros2 topic echo /ackermann_canopen_hardware/status
```

检查重点：

1. 控制器在 `inactive` 还是 `active`，确认不是“已加载就等于正在控制”。
2. 期望的四个命令接口是否存在，并查看当前是否被 controller claimed。
3. state interfaces 是否存在；实际状态由 `read()` 更新，而不是 command target。
4. CANopen 模式下硬件插件是否已收到反馈、状态和模式；arm 状态与控制器 active 状态应分别检查。

## 12. 一页式调用顺序

```text
启动：
launch → Xacro/robot_description → ControllerManager/ResourceManager
→ hardware plugin configure → export state/command interfaces → hardware start
→ pluginlib load controller → init → on_configure
→ query interface configurations → claim/loan → assign_interfaces → on_activate

每周期：
ResourceManager::read → SystemInterface::read → state 数值刷新
→ active Controller::update → command interface set_value
→ ResourceManager::write → SystemInterface::write → command 缓存
→ CANopen 后台 tick → target service → MotionControllerDriver → 电机

反馈：
电机 → MotionControllerDriver feedback topics → CANopen 插件 ROS callback
→ feedback cache → SystemInterface::read → state interfaces → controllers/broadcaster

停用/关闭：
controller switch → on_deactivate → release loans（硬件通常仍启动）
或进程关闭 → ResourceManager stop components → SystemInterface::stop
实机 disarm 是独立服务动作，不能与上述生命周期动作混为一谈。
```
