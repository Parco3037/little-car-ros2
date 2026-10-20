# 家庭环境操作手册

## 永久安全规则

1. 键盘不直接控制底盘，只发布 `/raicom/teleop_cmd`。
2. `cmd_vel_guard` 必须先启动并保持在线，才可启动键盘节点。
3. 看门狗超过 150 ms 未收到输入，持续向 `/controller/cmd_vel` 发布零速度；输入和输出队列均只保留最新一条速度状态。
4. 建图时速度上限为平移 0.10 m/s、转动 0.35 rad/s。
5. 键盘节点没有任何机械臂或夹爪快捷键。
6. 在车轮未架空并确认方向前，不得进行非零速度测试。

## 键位

| 键 | 动作 |
|---|---|
| W / S | 前进 / 后退 |
| A / D | 麦轮左移 / 右移 |
| Q / E | 左转 / 右转 |
| 空格 | 立即停车 |
| M | 请求启动固定场景任务（只调用门控服务，不直接控制舵机） |
| Ctrl+C | 发布零速度并退出 |

每个方向键是 200 ms 安全脉冲；持续按住时依靠键盘重复刷新。SSH 或键盘节点断开后，看门狗独立停车。

## 安全建图与导航入口

厂家原始 `slam.launch.py` 和 `navigation.launch.py` 会再次启动机器人驱动与 `init_pose`，家庭阶段不得直接使用。固定使用：

- 建图：`ros2 launch raicom_control home_slam.launch.py`
- 导航：`ros2 launch raicom_control home_navigation.launch.py`

这两个入口只在现有基础 ROS 上叠加 SLAM 或 Nav2，不会重复启动底盘、机械臂初始姿态或旧手柄。

保存地图时使用固定输出名：

`ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/ros2_ws/src/raicom_control/maps/home_mock_v1`

生成的 PGM/YAML 必须复制回 D 盘本仓库并提交。`home_navigation.launch.py` 在地图不存在时会直接拒绝启动。

## 家庭任务的默认状态

`home_mock_v1.yaml` 中目标坐标、到点动作和夹爪标定全部默认关闭。因此误调用 `/raicom/home/start` 只会返回失败，不会导航或动作。

完成家庭建图并记录 `home_pick_station` 后，只把 `goal_configured` 改为 `true` 即可进行纯导航验收；此时保持 `arrival_action_enabled: false`，到点后只稳定停车，不发布任何舵机命令。

Aurora930 恢复顺序：先正常关机并断开主电源，在断电状态下重新锁紧 USB
Wafer 线两端；冷启动后确认设备仍以 USB 2.0/`480M` 枚举（这是该型号的
正常规格），再启动驱动并检查 `/ascamera/camera_publisher/depth0/image_raw`
是否有首帧。若仍出现 `error -71` 或无首帧，优先更换短的可靠数据线、检查
5V 供电能力（规格为 1.5A）和接口，不继续反复重启 ROS 节点。

只有定位验收和 ID10 小幅开合标定也完成后，才能继续打开到点动作门禁：

- `arrival_action_enabled: true`
- `gripper_calibrated: true`

即便门禁打开，任务也必须同时满足定位新鲜、关节状态新鲜、Nav2 action 在线，且只有 Nav2 返回 `SUCCEEDED` 并停车 500 ms 后才会只操作 ID10。任何失败都发布零速度并跳过夹爪动作。
