# smarthome2026

智能家居 2026 线上赛机器人集成工程。项目目标是在同一个 ROS 2 Humble 工作空间内，把导航、视觉、任务状态机和上下位机通信合在一起，让机器人完成 A/B/C/D/E/F 区域导航、D 区识别抓取、E 区分类投放、F 区结束/充电等流程。

核心原则：

> 全工程只允许 `smarthome_comm` 直接打开下位机串口。导航、视觉、任务状态机都不直接访问 `/dev/tty*`，只通过 ROS topic/service 与 `smarthome_comm` 交互。

这样可以避免导航串口、视觉串口、机械臂串口互相抢设备，也方便在通信层统一增加断线重连和协议调试。

## 比赛任务抽象

| 区域 | 任务含义 |
|---|---|
| A | 起点 / 入口 |
| B | 导航得分点 |
| C | 导航得分点 |
| D | 物品识别与抓取区 |
| E | 分类投放区 |
| F | 终点 / 充电区 |

推荐流程：

1. 从 A 区启动。
2. 自主导航到 B、C、D 区。
3. 在 D 区启动视觉识别，获取物品类别和 3D 位姿。
4. 通过 `smarthome_comm` 发送机械臂抓取和夹爪控制命令。
5. 导航到 E 区，按物品类别完成分类投放。
6. 导航到 F 区，比赛流程结束。

## 软件架构

```text
Nav2 /cmd_vel --------------------------\
                                         \
Vision /detected_target --> adapter --> /smarthome/object_target
                                          \
TaskManager /smarthome/task/state --------> smarthome_comm --> Serial --> Lower MCU
TaskManager arm/gripper services --------/
Lower MCU state/ack --------------------/
```

| 模块 | 作用 |
|---|---|
| `pb2025_nav_bringup` | SLAM、定位、路径规划、速度输出 |
| `smarthome_vision` | 图像输入、目标检测、3D 位姿估计 |
| `smarthome_comm` | 唯一串口通信出口，统一发送底盘、视觉、机械臂、夹爪、到点标志 |
| `smarthome_task_manager` | A/B/C/D/E/F 任务流程状态机 |
| `smarthome_bringup` | 总启动入口 |

## 通信融合逻辑

本工程采用类似“任务状态驱动通信”的融合方式：

1. 导航只负责到点和输出 `/cmd_vel`。
2. 视觉只负责检测和发布目标，不再直接打开串口。
3. 任务状态机根据比赛流程推进导航、抓取和投放。
4. `smarthome_comm` 订阅导航速度、视觉目标、任务状态，并统一打包成串口帧。
5. 串口断开、重连、发送失败等问题全部由 `smarthome_comm` 处理。

## 串口帧格式

所有多字节字段均为小端序。

| 字段 | 类型 | 说明 |
|---|---|---|
| `magic` | `uint16` | 固定字节 `A5 5A` |
| `version` | `uint8` | 当前为 `1` |
| `seq` | `uint8` | 上位机递增序号 |
| `cmd_id` | `uint16` | 命令 ID |
| `payload_len` | `uint16` | payload 长度 |
| `payload` | `uint8[]` | 命令数据 |
| `crc16` | `uint16` | CRC16-Modbus |

## 上位机发送给下位机的数据

| cmd_id | 名称 | 来源 | payload | 作用 |
|---:|---|---|---|---|
| `0x0001` | `HEARTBEAT_TX` | `smarthome_comm` 定时器 | `uint32 time_ms` | 上位机心跳 |
| `0x0101` | `CHASSIS_VEL` | `/cmd_vel` | `float vx, float vy, float wz` | 底盘速度控制 |
| `0x0201` | `VISION_TARGET` | `/smarthome/object_target` | `uint8 class_id, uint8 source, float x, y, z, score` | 视觉识别目标 |
| `0x0301` | `GRIPPER` | `/smarthome/comm/gripper` | `uint8 open` | 夹爪开合 |
| `0x0302` | `ARM_COMMAND` | `/smarthome/comm/arm_command` | `uint8 command, uint8 class_id, float x,y,z,qx,qy,qz,qw` | 机械臂动作 |
| `0x0401` | `ESTOP` | `/smarthome/comm/set_estop` | `uint8 estop` | 急停 / 解除急停 |
| `0x0501` | `NAV_EVENT` | `/smarthome/task/state` 或 `/smarthome/navigation/reached_zone` | `uint8 zone_id` | 通知下位机当前到达区域 |

## 到点标志位

`NAV_EVENT` 只发送 1 个字节的 `zone_id`，不额外发送 `event_code`。

| zone_id | 区域 |
|---:|---|
| `1` | A |
| `2` | B |
| `3` | C |
| `4` | D |
| `5` | E |
| `6` | F |

示例：

| 场景 | 串口命令 |
|---|---|
| 到达 B 区 | `cmd_id=0x0501, payload=02` |
| 到达 D 区 | `cmd_id=0x0501, payload=04` |
| 到达 F 区 | `cmd_id=0x0501, payload=06` |

`smarthome_comm` 会自动监听 `/smarthome/task/state` 中的状态文本，例如 `NAV_DONE_D_STATUS_4`，解析出 D 区后发送 `zone_id=4`。调试时也可以手动发布：

```bash
ros2 topic pub --once /smarthome/navigation/reached_zone std_msgs/msg/String "{data: 'D'}"
```

## 串口断线重连

`smarthome_comm` 支持串口断开后自动重连：

- 启动时串口不存在，节点不退出。
- 按 `serial_reconnect_interval` 周期重新打开串口。
- 读写异常时关闭当前句柄，进入重连状态。
- 串口断开时，service 请求返回失败，topic 命令丢弃并打印日志。
- 串口恢复后自动继续发送后续命令。

关键参数：

```yaml
serial_device: "/dev/ttyACM0"
baudrate: 115200
fake_mode: false
serial_reconnect_interval: 1.0
```

## 启动方式

建图调试：

```bash
cd smarthome_2026_integrated_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch smarthome_bringup online_competition.launch.py \
  slam:=True \
  use_robot_state_pub:=True \
  use_rviz:=True \
  fake_comm:=True
```

实车比赛：

```bash
cd smarthome_2026_integrated_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch smarthome_bringup online_competition.launch.py \
  slam:=False \
  use_robot_state_pub:=True \
  use_rviz:=True \
  fake_comm:=False \
  serial_device:=/dev/ttyACM0 \
  baudrate:=115200
```

## 调试命令

观察上位机发送的串口帧：

```bash
ros2 topic echo /smarthome/comm/raw_tx
```

观察下位机状态：

```bash
ros2 topic echo /smarthome/lower_state
```

测试到点标志：

```bash
ros2 topic pub --once /smarthome/navigation/reached_zone std_msgs/msg/String "{data: 'E'}"
```

测试夹爪：

```bash
ros2 service call /smarthome/comm/gripper example_interfaces/srv/SetBool "{data: false}"
```

## 回档策略

建议所有集成通信改动都推送到独立分支，例如：

```text
codex/unified-serial-comm
```

不要直接覆盖 `main`。这样原来的代码仍保留在 `main` 和 GitHub 提交历史中。如果新通信方案需要回退，可以直接切回 `main`，或者 revert 该分支对应的合并提交。
