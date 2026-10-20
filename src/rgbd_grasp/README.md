# RGB-D 单次蓝块抓取

入口为 `ros2 launch rgbd_grasp rgbd_grasp.launch.py`。它针对 Aurora930 和当前 LanderPi 机械臂执行一次蓝块抓取并保持，不包含自动定点放置或连续分拣。

## 启动条件

先加载 ROS Humble、合法取得的厂商工作空间和本仓库构建环境。相机驱动 `deptrum-ros-driver-aurora930` 必须已安装，RGB、深度和 CameraInfo 的配准、内参和单位须正确。

默认启用运动、自动准备和自动抓取。底层硬件、舵机转发和完整机器人描述默认不启动，复用已有服务。相机默认启动；若相机已运行，应设 `start_camera:=false`，当前代码不会自动检查重复驱动。即使关闭相机启动，launch 仍会查找相机包路径。

先检查感知：

```bash
ros2 launch rgbd_grasp rgbd_grasp.launch.py motion_enabled:=false auto_prepare:=false auto_grasp:=false
```

完成本机标定与现场检查后，默认命令会自动运动：

```bash
ros2 launch rgbd_grasp rgbd_grasp.launch.py
```

## 抓取流程

观察姿态 → 等待新鲜蓝块观测 → 必要时有界左右搜索 → ID1 水平对准 → 全路径 IK/FK 预检 → 开夹、接近、闭爪 → 收拢保持。成功或拒绝后均不自动抓第二块。

识别节点发布相机光学坐标。`target_to_ik` 使用真实关节读回和手眼矩阵换算为机械臂坐标，并保留新鲜度与运动阶段门控。抓取前需要稳定观测、关节与工作空间条件及 IK/FK 路径检查全部通过。

默认选块规则为 `nearest`：先选择最近深度组，再选择该组中较上方的蓝块；可显式设置 `target_selection:=min_v` 按图像高度选择。对准和抓取使用相同规则，但这不证明多个紧邻方块一定能分离识别。

## 当前设备参数

| 参数 | 默认值与含义 |
|---|---|
| `grasp_height_offset_m` | 0.015 m，抓取高度偏移；允许 0 至 0.020 |
| `sag_compensation_m` | 0.020 m，下垂补偿基值；实际随伸展距离变化，并受 IK 可达性限制 |
| `pregrasp_clearance_m` | 0.015 m，预抓及前半段接近的附加净空 |
| `grasp_id4_offset_pulse` | 60，抓取接近阶段的 ID4 脉冲补偿；允许 0 至 60 |
| 图像水平对准 | 中心 326 px、容差 6 px，单步最大 20 脉冲，接受范围 320 至 332 px |

ID4 脉冲补偿是关节命令补偿，不是以毫米计的空间抬高。日志中的 `grasp_target`、`commanded_fk_xyz`、请求下垂补偿和最终可行补偿含义不同，模型预测不等于负载下实测。

观察姿态为 ID1=450、ID2=800、ID3=50、ID4=250、ID10=390。保持姿态为 ID1=450、ID2=500、ID3=250、ID4=400、ID5=860。保持阶段不另发 ID10 张爪指令，保留原闭爪命令。上述姿态、手眼矩阵和方块中心补偿均需根据自己的设备复核。

## 状态、停止与再次抓取

```bash
ros2 topic echo /grasp_once_node/status
ros2 service call /grasp_once_node/stop std_srvs/srv/Trigger '{}'
```

停止取消后续阶段，不主动张爪。闭爪读回可以拒绝空闭合，但不能证明物体夹稳。`holding_unconfirmed` 和 `auto_grasp_complete` 明确保留 `physical_holding_confirmed=false`，需要现场确认。

保持期间再次 prepare/grasp 会被占用锁拒绝。要开始新一轮需明确调用 release；release 只开夹，不会自动移动到放置点。

检测节点可在用户目录保存 ready 阶段最近的同步 RGB、mask、深度和候选记录。`selection_record_dir` 为空时关闭记录。这些运行数据不属于公开源码，应留在本机。

## 验证边界

本次发布保持六个固定抓取模块和 launch 的原始字节，与工作台 `depth_pick_release.py` 中的摘要一致。打包依赖、许可证与说明做了公开发布适配，未改抓取动作逻辑。离线测试、ROS 隔离集成与实物夹取必须分别验收，详见仓库 `docs/VALIDATION.md`。
