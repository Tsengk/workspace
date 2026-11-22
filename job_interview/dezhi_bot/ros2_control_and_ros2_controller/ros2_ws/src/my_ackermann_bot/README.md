# 阿克曼底盘：Gazebo、mock CANopen 与实车

本工程保留 ROS 2 Foxy，不依赖该分支中不存在的 `canopen_ros2_control` 包。
新增 `ackermann_canopen_hardware/AckermannCanopenSystem` 硬件插件，通过后台 ROS
服务/话题连接当前 `ros2_canopen` 的 `MotionControllerDriver`。

关于 ros2_control 组件职责、接口暴露/借用、Controller 与硬件生命周期以及完整函数调用时序，见
[生命周期与调用时序文档](docs/ros2_control_lifecycle_and_call_sequence.md)。

```text
Twist → AckermannController → position/velocity interfaces
                              ↓
                    AckermannCanopenSystem
                    read/write ↔ 有界缓存
                              ↓ 后台执行器
                    MotionControllerDriver
                              ↓ Lely / CANopen
                    SocketCAN → 电机驱动器
```

这是功能接入和低速调试桥接，不是经过认证的实时或安全控制系统。ROS 服务往返并不
保证四个轴同步；`target` 成功只表示驱动软件接受目标，不表示实际运动到位。需要严格
同步周期时，应保留关节接口和标定配置，替换后台服务桥接为经过验证的直接 PDO 后端。

## 1. 构建

在 `ros2_ws` 目录执行，限定源码路径以避开未适配 Foxy 的上游阿克曼控制器源码：

```bash
source /opt/ros/foxy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
  --base-paths src/ros2_canopen src/ackermann_canopen_hardware src/my_ackermann_bot \
  --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

首次构建 Lely 需要联网。不要同时运行两套示例或两个实车主站，它们使用相同的
`/controller_manager` 和电机节点名称。

## 2. 仿真与实机切换

统一入口是 `ackermann.launch.py`，默认 `mode:=gazebo`：

| mode | ros2_control 硬件 | 电机后端 | 时钟 | 阿克曼控制器默认状态 |
|---|---|---|---|---|
| `gazebo` | 原 GazeboSystem | Gazebo Classic | 仿真时间 | active |
| `mock` | AckermannCanopenSystem | 本地假 CANopen 服务 | 系统时间 | active，但硬件未解锁 |
| `canopen` | AckermannCanopenSystem | 真正 ros2_canopen 主站 | 系统时间 | inactive |

Gazebo 模式保持原有入口和行为：

```bash
ros2 launch my_ackermann_bot ackermann.launch.py mode:=gazebo
```

原命令 `ros2 launch my_ackermann_bot gazebo_ackermann.launch.py` 也可继续使用。
Gazebo 不加载电机映射，不访问 CAN，也不需要硬件解锁。

### 不接电机验证完整软件链路

```bash
ros2 launch my_ackermann_bot ackermann.launch.py mode:=mock enable_motion:=true
```

mock 模式不会打开 `can0` 或 `vcan0`，只仿真相同的服务和反馈接口；并不是车辆物理仿真，
也不能替代 CANopen 总线、PDO、CiA402 状态机或真实驱动器验证。假驱动器初始处于
Operation Enabled，并使用独立的 `mock_canopen_motors.yaml` 标定。

等待两个控制器 active 后，在另一个已 source 的终端执行：

```bash
ros2 control list_controllers
ros2 service call /ackermann_canopen_hardware/arm std_srvs/srv/Trigger "{}"
ros2 topic pub -r 10 /ackermann_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}"
```

通过 `/joint_states` 观察关节反馈。Ctrl-C 停止发布后，控制器在指令超时后请求驱动零速，
桥接层按加速度限制减速，保留转向目标。显式停机：

```bash
ros2 service call /ackermann_canopen_hardware/disarm std_srvs/srv/Trigger "{}"
ros2 topic echo /ackermann_canopen_hardware/status
```

`disarm` 向所有轴请求 halt。mock 驱动随之进入 Quick Stop 状态；重新测试可重启 mock
bringup，或在无运动指令时恢复假驱动并重新激活控制器，再显式 arm。

## 3. 实车执行机构前提

当前插件严格要求以下顺序的四路独立执行机构：

1. `front_left_steer_joint`：位置命令，Profile Position，模式 1。
2. `front_right_steer_joint`：位置命令，Profile Position，模式 1。
3. `rear_left_wheel_joint`：速度命令，Profile Velocity，模式 3。
4. `rear_right_wheel_joint`：速度命令，Profile Velocity，模式 3。

每个关节均导出实际 position 和 velocity 状态。节点名称可以通过电机映射配置更改，
但不支持把两个关节静默映射为同一台电机。如果是一台转向电机带连杆，或者单台驱动
电机带机械差速器，需要先修改控制器/传动模型，不能直接使用这套四轴接线。

驱动器必须真正支持 CANopen CiA402。厂家私有 CAN 协议、J1939 等不能直接使用本驱动。
本版本只接入 PP/PV 模式，不自动选择 CSP/CSV，不自动初始化、回零、使能或清故障。

## 4. 将 CANopen 适配到真实驱动器

### 4.1 准备 EDS、节点 ID 和总线参数

从厂家取得对应型号/固件的 EDS 或 DCF，确认 PP/PV 模式、单位定义、限位和回零策略。
配置 CAN 适配器为 SocketCAN，按实际设备设置接口、波特率、终端电阻与接地；启动脚本
不会自动执行 `ip link`、修改网络或使能电机。不能根据模板猜测波特率或编码器单位。

复制 `config/canopen_bus.template.yml` 到自己的配置目录。模板中的 master ID=1、电机
ID=2/3/4/5 只是示例，必须改为真实值且不重复。四个电机的 YAML 键应对应映射文件中的
`driver_node` 名称，例如 `front_left_steer_motor` 对应 `/front_left_steer_motor`。

把两个 `REPLACE_WITH_...eds` 改成真实 EDS 文件名或绝对路径。根据厂家说明调整 PDO
数量、映射、传输类型和 SDO 配置；不能假定所有驱动器都允许模板中的动态映射：

| 方向，以驱动器为准 | 对象 | 用途 |
|---|---|---|
| TPDO | 0x6041、0x6061 | 状态字、实际工作模式 |
| TPDO | 0x6064、0x606C | 实际位置、实际速度 |
| RPDO | 0x6040、0x6060 | 控制字、目标工作模式 |
| RPDO | 0x607A、0x60FF | 目标位置、目标速度 |

模板 SYNC=20000 微秒、驱动 period=20 毫秒，仅为 50 Hz 示例。需要核对总线负载、PDO
行为和驱动器能力，不能只把 controller_manager 的 update_rate 调高就认为总线同频。
厂家要求的轮廓速度、加减速、软限位、Quick Stop、heartbeat / RPDO 超时等参数，应在
厂家工具或经审核的启动 SDO 中配置。不要猜测或批量写入未知的对象字典。

生成运行配置，例如：

```bash
ros2 run ackermann_canopen_hardware prepare_canopen_config.py \
  --input /absolute/vehicle_config/bus.template.yml \
  --output-dir /absolute/vehicle_config/generated
```

工具验证节点 ID、驱动类型和 EDS 是否存在，复制 EDS，更新绝对 `dcf_path` 并调用
`dcfgen` 生成 `bus.yml`、`master.dcf` 和相关 bin 文件。它不验证物理接线、单位或设备
实际支持情况。每次更改 EDS/PDO/总线配置后重新生成；不要使用 mock slave 的 EDS
替代真实厂家文件。若生成 `master.bin`，启动时通过 `master_bin` 传入；没有则留空。

### 4.2 标定关节单位

复制 `config/canopen_motors.yaml`，逐轴填写真实参数。默认转换系数 1.0 是占位值，
`calibration_confirmed: false` 会阻止解锁；不要只将该标志改为 true 而跳过标定。

```text
native_position = position_offset + direction × position_scale × joint_position_rad
native_velocity = direction × velocity_scale × joint_velocity_rad_per_second

joint_position_rad = (native_position - position_offset) / (direction × position_scale)
joint_velocity_rad_per_second = native_velocity / (direction × velocity_scale)
```

| 参数 | 含义 |
|---|---|
| `driver_node` | 对应真实 CANopen 电机 ROS 节点的绝对名称 |
| `position_scale` | 每个关节 rad 对应的驱动器位置单位，必须正数 |
| `velocity_scale` | 每个关节 rad/s 对应的驱动器速度单位，必须正数 |
| `position_offset` | 关节零位对应的驱动器原始位置 |
| `direction` | +1 或 -1，分别统一命令和反馈的方向 |
| `min_position` / `max_position` | 转向关节的实际机械允许范围，单位 rad |
| `max_velocity` | 转向目标变化速率上限 rad/s，或驱动轮角速度上限 rad/s |
| `max_acceleration` | 驱动轮目标变化率上限 rad/s²；转向当前仅使用速度限制 |
| `operation_mode` | 转向 1，驱动轮 3，必须与实际模式反馈一致 |

若位置单位是电机端每转 N 个计数、减速比 G=电机转数/关节转数，则线性传动可取
`position_scale = N × G / (2π)`。若速度单位恰好是电机 rpm，则
`velocity_scale = 60 × G / (2π)`；这只是条件成立时的公式，必须按厂家定义核实。
转向连杆非线性或驱动器电子齿轮/单位缩放会改变关系，本实现的线性映射不能冒充非线性标定。

完成架空单轴标定后，再将自己配置文件中的 `calibration_confirmed` 改为 true。
另复制 `ackermann_controllers_real.yaml`，把轴距、轮距、轮径和速度限制改成实车数据。
当前 0.32/0.26/0.05 m 是原仿真尺寸，并非任何真实车辆的标定结果；URDF 几何也应一致。

### 4.3 实车启动与解锁

先在车轮架空、具备独立急停的条件下以禁止运动模式启动：

```bash
ros2 launch my_ackermann_bot ackermann.launch.py mode:=canopen \
  motor_config:=/absolute/vehicle_config/motors.yaml \
  controllers_file:=/absolute/vehicle_config/controllers.yaml \
  bus_config:=/absolute/vehicle_config/generated/bus.yml \
  master_dcf:=/absolute/vehicle_config/generated/master.dcf \
  can_interface:=can0 enable_motion:=false
```

此时阿克曼控制器默认只加载、配置，不激活。硬件插件等待四个节点反馈和实际 SDO
状态读取成功，默认最多 15 秒；没有总线/节点会启动失败，不会用命令值假冒反馈。
启动超时可在电机 YAML 中调整。即使禁止 target 发送，停机/故障仍会请求 halt，
所以本软件不是完全不改变设备状态的诊断工具。
启动助手会等待 controller_manager 实际响应，再顺序加载两个控制器，避免旧版 Foxy
spawner 在硬件仍等待总线反馈时提前超时；助手默认等待上限为 60 秒。

在厂家批准的顺序下完成初始化、必要的回零、PP/PV 模式设置和状态确认。现有驱动提供
`/<motor>/init`、`position_mode`、`velocity_mode`、`recover` 服务，但 `init` 可能触发
真实回零运动，不能无条件批量调用。旧 Foxy 驱动的 init/recover 返回值不能可靠表示
设备完成初始化；必须核对 0x6041 状态字、0x6061 模式和实际反馈。

完成全部检查后停止 bringup，以相同参数和 `enable_motion:=true` 重启。程序仍然不会
自动运动或自动 arm。保持 Twist 发布停止，确认四个轴实际就绪，然后执行：

```bash
ros2 control switch_controllers --start ackermann_controller --strict
ros2 service call /ackermann_canopen_hardware/arm std_srvs/srv/Trigger "{}"
```

激活控制器会以实际转向位置初始化目标，后轮目标为零。arm 还要求当前控制循环在更新、
四轴状态新鲜且 Operation Enabled、模式分别为 1/1/3/3、后轮静止、转向命令接近实际位置。
通过后才允许遥控发布 Twist。`start_controller:=true` 可显式改变默认加载策略，但不
绕过标定、状态检查或 arm。回零导致位置改变后，应在无运动指令时重新激活控制器。
停止/重启会请求 halt，驱动器可能不再处于 Operation Enabled。最终运行实例中必须
重新核对状态，必要时按厂家批准的流程恢复和设置模式，不能假定前一次初始化仍然有效。

## 5. 安全边界与故障处理

- 三道软件门槛：`enable_motion:=true`、真实标定确认、显式 arm，缺一不可。
- `read/write` 只用 try-lock 交换固定大小缓存，不在控制线程等待 ROS 服务或 SDO。
- 每轴最多一个在途 target 请求和一个在途 SDO 读取。旧 Foxy 无请求取消接口，服务
  超时即锁存故障并停止发新目标，避免无限堆积；故障修复后必须重启 bringup。
- 后台按关节限位、转向速率和后轮加速度限制目标；驱动器自身限制仍需独立配置。
- 控制器指令超时会请求后轮零速、保留转向目标；控制循环停更、反馈过期、SDO 失败、
  模式/状态错误、目标拒绝或非法数值会锁存故障，并请求四轴 halt。
- halt 只是尽力而为的通信请求，不代表车辆已经停住。旧驱动的 halt 响应 success
  也不可靠，不能将其当作停车确认。通信中断、主机掉电或程序被强杀时只能依赖驱动器
  自身的 watchdog、经过验证的 Quick Stop/制动策略及独立急停/STO。
- 现有 actual_position/actual_speed 话题没有驱动器采样时间戳，可能重复发布缓存值。
  桥接层校验 ROS 接收新鲜度并另做实际 SDO 活性检查，但无法证明每条反馈来自最新 TPDO。
  生产系统应补充原始 PDO 时间戳、设备心跳和经过验证的通信故障处理。
- 一台转向电机的状态异常应禁止整车继续驱动；不能依赖仅驱动轮零速来替代机械安全设计。
- 本实现尚未增加底盘里程计与 odom TF；joint_states 不等于完整导航反馈链路。

## 6. 自动测试

```bash
colcon build --packages-select ackermann_canopen_hardware my_ackermann_bot \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ackermann_canopen_hardware my_ackermann_bot \
  --ctest-args -R 'test_ackermann_canopen_system|test_mock_bringup' \
  --return-code-on-test-failure
colcon test-result --verbose
```

`my_ackermann_bot` 还提供 `test_mock_bringup`，会在独立 ROS domain 启动 mock 完整链路，
验证控制器激活、解锁、轮角限幅、前进/倒车/直行反馈、指令超时和 disarm。可在该包启用测试后
运行 `colcon test --packages-select my_ackermann_bot --ctest-args -R test_mock_bringup`。

测试使用本机虚拟服务，验证配置拒绝、禁止运动、未标定、模式错误、正反向单位转换、
显式停机、SDO 失败、非法目标、控制循环超时、反馈超时和目标拒绝，不访问真实 CAN。
软件测试通过并不代表实机接线、驱动器配置或车辆安全已经通过验收。
