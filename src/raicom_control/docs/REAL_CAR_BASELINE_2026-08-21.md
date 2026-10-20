# RAICOM2026 实车基线（2026-08-21）

本记录来自树莓派当前运行态的只读盘点。它证明实施前状态，不代表后续阶段验收通过。

## Docker 基线

| 容器 | 镜像 | 状态 | 网络/端口 | 结论 |
|---|---|---|---|---|
| `MentorPi` | `ros:humble` | 运行 | host | 保留，ROS2 主环境 |
| `pi_cam_env` | `yolo_camera:v1` | 运行 | bridge，8000/8080 | 保留，SCI 相机链后续使用 |
| `yolo_env` | `yolo_infer:v1` | 运行 | 共享 `pi_cam_env` 网络 | 保留，YOLO 后续使用 |
| `objective_dhawan` | `ros:humble` | 停止约 9 个月 | host | 永久停用 |

实施前容器总数为 4。不得改变数量、名称和镜像 ID。

## 主机与网络

- SSH：`pi@<ROBOT_OR_HOST_IP>:22`
- 主机 ROS 自动启动：`start_app_node.service` 已启用且运行。
- 当前监听：22、8000、8080；8081 尚未监听。
- `w8872` 与 `HW-6770556A` 均已保存并设为自动连接，实施前优先级均为 0。
- 根分区：57 GiB，总使用约 50 GiB，可用约 4.4 GiB，使用率 92%。
- 本地任务文件固定放在 `<LOCAL_WORKSPACE>`，不得写入或下载到 C 盘。

## ROS2 在线能力

- 底盘：`/controller/cmd_vel`，麦轮车型 `LanderPi_Mecanum`。
- 里程计与融合：`/odom_raw`、`/odom`、EKF、IMU。
- 激光雷达：MS200，设备 `/dev/ldlidar -> /dev/ttyUSB0`，话题 `/scan_raw`、`/scan`。
- 机械臂：`/arm_controller/follow_joint_trajectory`。
- 夹爪：`/gripper_controller/follow_joint_trajectory`；底层另有总线舵机控制话题。
- Web：ROS bridge 在线；相机 Web 端口 8000，ROS 原 Web 使用 8080。
- Aurora930：ROS2 深度相机链保留；SCI 第一阶段不使用。
- 现有应用：巡线、目标跟踪、AR、手势与手轨迹节点在线。

## 已确认故障

- 厂家 `joystick_control` 在没有兼容手柄时进入忙循环，占用约 99.6% CPU，并持续写错误日志。
- `/joy_node` 与 `/joystick_control` 被 `bringup` 默认启动。
- `slam/launch/include/robot.launch.py` 的 `use_joy` 默认值为 `true`；顶层 SLAM 和 Navigation 未显式关闭它。
- 厂家键盘程序不能作为安全方案：麦轮停止超时后仍可能保留线速度，且含舵机控制快捷键。
- 底盘控制本身没有 250 ms 指令超时，因此必须由独立 `cmd_vel_guard` 持续发布停车指令。

## 既有修改保护

`/home/ubuntu/ros2_ws` 父仓库在实施前已经很脏，不得整体回退。已确认与本次相关的既有修改：

- `src/slam/launch/include/robot.launch.py` 中 `scan_topic` 已由厂家基线的 `scan_raw` 改为 `scan`。

本次修改必须保留该变化。

### 目标文件实施前 SHA256

| 文件 | SHA256 |
|---|---|
| `src/bringup/launch/bringup.launch.py` | `27a3ef48082c2889ba33ef9e458e9799a36f794e3fa210d59cf915a7773c8fb2` |
| `src/slam/launch/slam.launch.py` | `1e61ae08fc868021bd0222b4269357bbd3bb06cc3715884b9ab94d2e562f38b1` |
| `src/slam/launch/include/robot.launch.py` | `67634236434be30f6a59dbca3e7b11b5086b5ad65ba93ecb539794530e92bc90` |
| `src/navigation/launch/navigation.launch.py` | `4c7ce6aaddca5f52fd36cbf8ff1173e649ae8709bcc1ae40cf9a10b7a0d17dba` |
| `src/peripherals/launch/joystick_control.launch.py` | `631f479eeac3340f762b2e8a99e2f9a69bebb1a7354e09992e6ee478f8c09b5c` |
| `src/peripherals/peripherals/joystick_control.py` | `23a9acc425f7e506fce3995ce45416b7ea24e935e61d52170e1cc3e9659bce11` |

## 安全停止点

在现场确认车轮架空或四周净空前，只允许：只读盘点、文件备份、编译、单元测试、停止旧手柄节点和发布零速度。不得发布任何非零底盘速度，不得执行机械臂或夹爪动作。

