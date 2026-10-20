# 部署说明

本目录中的内容只适用于已经盘点过的当前实车，不得用于另一个系统镜像。

- `vendor_launch_disable_joy.patch`：只改变 `bringup/slam/navigation` 的手柄默认启动门禁；不删除厂家源码，并保留当前 `scan_topic=scan` 修改。
- `vendor_ms200_driver.patch`：将 `MS200` 分支恢复为现有 `oradar_lidar` 驱动，不再误用 LD19 驱动；部署时同时备份源码与安装文件。
- `vendor_mecanum_odom_xy.patch`：修复厂家里程计漏算麦轮 `linear_y` 的问题，按机体速度到里程计坐标的二维旋转公式同时积分 x/y。
- `vendor_mecanum_linear_calibration.patch`：依据旧系数下两次后退合计里程计 `0.401375 m`、实体尺量 `0.36 m`，以及中间系数下前进/后退各 `0.203 m`、实体尺量 `0.18/0.19 m`，将线速度里程计系数由 `0.94` 折中校正为 `0.770`；部署后必须再做双向距离复测。
- `vendor_robot_description_lightweight.patch`：厂家 robot-description 启动时优先加载 RAICOM 的无网格轻量 URDF；设置 `RAICOM_USE_LIGHTWEIGHT_URDF=false` 或恢复备份即可回到厂家 Xacro。该补丁只切换显示模型来源，不修改 joint、TF、控制器或运动学参数。
- `raicom_cmd_vel_guard.service`：在现有 `MentorPi` 容器内运行唯一一个独立速度看门狗，不创建容器。

部署前必须先核对 `docs/REAL_CAR_BASELINE_2026-08-21.md` 中的目标 SHA256，并创建时间戳备份。补丁必须先执行 `git apply --check`；不匹配时停止，不得强行套用。

回退顺序：

1. 停止 `raicom_cmd_vel_guard.service`，并发布一次零速度；
2. 从对应时间戳备份恢复四个厂家 launch 文件；
3. 删除新增包的源码和该包自己的 `build/install` 产物（不得清理整个工作空间）；
4. 保持 `objective_dhawan` 停止，不改变其他容器。

## 轻量 URDF 验收

1. 生成记录中的源模型 SHA256 必须为当前实车展开模型的哈希；不一致时重新生成，不沿用旧文件。
2. `landerpi_lightweight.urdf` 中 `<mesh>` 数必须为 0。
3. 生成器必须证明 link、joint、inertial、transmission 定义未改变，并通过隔离 ROS 域的 `robot_state_publisher` 解析。
4. 部署后 `/robot_state_publisher` 只能有一个实例，`/tf` 不得出现重复发布者；切换过程不启动底盘、舵机或夹爪。
5. 若外形包围盒影响后续碰撞规划，设置 `RAICOM_USE_LIGHTWEIGHT_URDF=false` 回退厂家模型；实际底盘 footprint 和机械臂标定仍以各自正式配置为准。
