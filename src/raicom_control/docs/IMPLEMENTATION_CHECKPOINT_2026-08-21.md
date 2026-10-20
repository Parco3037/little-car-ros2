# RAICOM2026 家庭实施检查点（2026-08-21）

## 当前结论

软件与安全隔离、MS200 驱动修复、麦轮横移里程计修复和家庭建图已经完成，`home_mock_v1` 已保存并由 Nav2 地图服务器重新加载。当前底盘持续收到零速度；定点导航、机械臂动作、夹爪动作和 Aurora930 抓取尚未验收。

现场人员已确认地面测试区域四周净空。W/S/A/D/Q/E 六个方向已经逐项进行低速实车验证，左右对称旋转后实体回正；当前进入固定点导航阶段。

## Git 回退点

本地独立仓库：`<LOCAL_WORKSPACE>\raicom_control`

| 提交 | 内容 |
|---|---|
| `4740be0` | 实施前实车基线 |
| `4c6ebec` | 安全键盘、看门狗、家庭任务门禁 |
| `3926d4b` | 旧手柄启动补丁与 systemd 守护服务 |
| `99aa37a` | 修复 ROS 清单合法性 |
| `9036581` | 修复服务终止前停车路径 |
| `f621ba8` | 不重复启动驱动的家庭 SLAM/Nav2 入口 |
| `0c5cc3e` | 记录正向实测与里程计尺度偏差 |
| `225ba56` | 修复 MS200 启动驱动映射 |
| `39c43d7` | 新增限速、限半径、遇障停车的建图运动节点 |
| `92ddf45` | 增加 SLAM 跟踪保护与横移短验模式 |
| `7218288` | 修复厂家麦轮横移里程计积分 |

树莓派双份备份：

- 容器：`/home/ubuntu/raicom_backups/20260821T0125_pre_raicom`
- 主机：`/home/pi/raicom_backups/20260821T0125_pre_raicom`
- 已备份 12 个目标文件，包含源码和安装目录的四个启动文件。

回退时不得整体重置脏的厂家仓库。只恢复备份中的四个源码启动文件与四个安装启动文件，停止并禁用 `raicom_cmd_vel_guard.service`，然后按本地 Git 提交选择性回退新增包。

## 已实施

- 新增 `raicom_control` ROS2 包，已被识别为 `ros.ament_python` 并构建成功。
- 新增 `keyboard_input`：W/S 前后、A/D 麦轮横移、Q/E 转向、空格停车、Ctrl+C 停车退出；无任何舵机快捷键。
- 新增独立 `cmd_vel_guard`：150 ms 超时、20 Hz 连续停车、输入/输出队列深度 1、平移上限 0.10 m/s、角速度上限 0.35 rad/s、非法数值归零。
- 新增 `/raicom/home/start` 和 `/raicom/home/cancel`；目标、到点动作和夹爪标定默认全部关闭。
- 到点任务仅允许 Nav2 `SUCCEEDED`、停车 500 ms 后操作 ID10，关节 1–5 不发送命令。
- 新增 `home_slam.launch.py` 与 `home_navigation.launch.py`，不重复启动底盘、机械臂 `init_pose` 或旧手柄。
- 家庭地图不存在时 Nav2 入口硬失败，未产生 Nav2 节点。
- `bringup/slam/navigation` 中 `use_joy` 默认设为 `false`，厂家手柄源码保留。
- 独立 systemd 服务 `raicom_cmd_vel_guard.service` 已启用且运行，实车上仅一个守护实例。
- `w8872` 自动连接优先级已设为 100；`HW-6770556A` 保留并设为 -10 回退入口，当前连接仍为 `w8872`。
- 厂家 `MS200` 分支已恢复为现有 `oradar_lidar` 驱动，`/scan` 约 10 Hz。
- 厂家里程计已按二维机体速度变换同时积分 `linear_x/linear_y`，不再漏掉麦轮横移。
- 建图运动跟踪改用 `/odom_raw`；SLAM 回环只修正地图，不再被误判为底盘瞬移。
- 新增 `nav_cmd_bridge`，将 Nav2 `/cmd_vel` 转发到 `/raicom/teleop_cmd`，再由独立看门狗限幅并输出 `/controller/cmd_vel`；导航速度不再绕过 150 ms 断控保护。
- 导航验证时恢复厂家里程计启动项的 `cmd_vel:=controller/cmd_vel` 重映射，里程计节点不再直接订阅 Nav2 `/cmd_vel`。
- 新增 `odom_tf_bridge`，把 `/odom_raw` 原样发布为 `/odom` 并用同一姿态广播 `odom→base_footprint`，避免厂家 EKF 将 IMU 航向与底盘里程计融合成相反方向。

## 实测证据

- 本地和容器内纯逻辑测试：4/4 通过。
- 隔离话题输入 `x=0.5, y=-0.5, z=1.0`，输出被夹到 `0.10, -0.10, 0.35`。
- 三次 150 ms 看门狗实车探针延迟为 `116.8/101.3/106.4 ms`；调整执行器频率后再次复测为 `112.6 ms`，均低于 300 ms，随后输出持续全零。
- 守护进程收到 TERM 后不再出现 ROS 上下文关闭异常。
- 家庭任务默认调用返回 `success=False, home_pick_station is not configured`，夹爪测试话题无消息。
- 键盘节点在交互终端中成功启动；未按方向键，仅 Ctrl+C，终端正常恢复，随后底盘输出仍为全零。
- 第一次正向脉冲使用 `linear.x=0.03`，里程计前进约 7 mm；停止后速度输出恢复全零。
- 第二次正向测试使用 `linear.x=0.06`、约 1.5 s；里程计增量约 9.62 cm，现场尺量约 7 cm，实测/里程计比约 0.73。该单次结果仅记录为待校准偏差，不据此直接修改轮径或里程计参数。
- 第二次正向测试结束后速度输出已恢复全零；现场确认移动方向为车头正前方。
- `10 cm` 左移并回原点短验成功，运动节点报告最终半径 `0.000 m`；现场确认实体横移。
- 完整建图 19/19 段成功，最高平移 `0.05 m/s`、最高角速度 `0.20 rad/s`、单向平移 `0.25 m`、最大允许半径 `0.50 m`，最终半径 `0.001 m`。
- `home_mock_v1` 为 `81 x 94` 像素、`0.05 m/pixel`；Nav2 `map_server`、AMCL、规划器、控制器和 BT Navigator 均进入 `active`，重新加载尺寸一致。
- 正式地图哈希：PGM `58bb76e62f289d1dcd7a0bdc406125b93513c7035119bfb8e215b59d48c346ea`；YAML `9fb1fcf3ff5994cd95c984a6b0224ff3508a5bc1ec2c9945bc4378f8f23a92df`。
- 旧线性系数 `0.94` 下，两次后退里程计合计 `0.401375 m`、实体尺量 `0.36 m`；中间系数 `0.843` 下，前进/后退里程计各约 `0.203 m`、实体尺量 `0.18/0.19 m`，确认存在方向差。
- 最终折中系数 `0.770` 下，前进和后退里程计均约 `20.2 cm`、实体尺量均为 `20.0 cm`，两向误差均不超过 `0.3 cm`。
- W/S/A/D/Q/E 六个方向均经现场人工确认正确；最后一次左转、停车、右转测试后实体回正，里程计残余航向约 `0.53°`。
- 厂家 `/joy_node` 和 `/joystick_control` 均已停止；99.6% CPU 忙循环消失。
- `objective_dhawan` 保持 `exited`；容器数量、名称和镜像不变。
- 候选点 `(-0.78, 0.10)` 在实时代价地图中通过纯规划，`ComputePathToPose` 返回 `SUCCEEDED`；该结果只证明路径可生成，不等于实车导航通过。
- 第一次实车 `NavigateToPose` 最终 `ABORTED`，累计 20 次恢复，不计入 5 次导航验收。失败后 Nav2 与 `nav_cmd_bridge` 已停止，独立看门狗持续输出全零，现场确认车辆已停住。
- `odom_tf_bridge` 先使用隔离话题 `/raicom/odom_bridge_test` 和隔离 TF `raicom_odom_test→raicom_base_test` 验证，输出位置 `(0.058, -0.057) m`、航向 `+40.325°`，与同一时刻静止的 `/odom_raw` 一致；隔离实例随后停止。
- 当前运行态已停止厂家 `ekf_node`，启用 `raicom_odom_tf_bridge`；`/odom` 发布者数量为 1，`/odom` 与 `odom→base_footprint` 均为 `(0.058, -0.057) m / +40.325°`。Nav2 节点未运行，`/controller/cmd_vel` 实测持续全零。
- 里程计桥本地纯逻辑测试累计 14 项通过；容器当前部署测试集 10 项通过。新节点部署文件与本地 SHA256 完全一致。

### 当前部署文件 SHA256

| 文件 | SHA256 |
|---|---|
| `cmd_vel_guard.py` | `2b94d140ccbda0c12c71d72e051e9796461bb6a7f21ab884fde97e6f881839d4` |
| `keyboard_input.py` | `b5679b2ead111b4877bb84a6e7a3fa9e558957e740a848c4c95b37fb16461b47` |
| `home_mission.py` | `7dd3cbb34c2a3f142e7abbb1bb960b5d54e407a25dce17f464297a7dbc95cc63` |
| `home_slam.launch.py` | `53ca997595e6ff6c9e46dfbb659305ccc86bf1225bc2c7c36b8a57e96b8ee5e2` |
| `home_navigation.launch.py` | `660e8864ae63294e4b78b491517c4d4e1ce263aba02311b659ded2c23bb29c52` |
| `nav_cmd_bridge.py` | `5d2e6d14d15ec557479dc78b15c93a1985073a490d1204d21596c3bc20caf1e0` |
| `odom_bridge_logic.py` | `a95262171641e24b627201627f036bc5f1668e7f47eda6afe3606162e141b403` |
| `odom_tf_bridge.py` | `43ca686c62fca7d3f266669ac3ec3d3ea0a65d2bac061961abce0b51a389d8f9` |

`home_navigation.launch.py` 加入导航安全桥后当前部署哈希为 `3c391c2e173b62e0d1ad76afab1acb6b5f3e9f5280e4ab676e24e2b097307888`；上表旧哈希保留用于追溯桥接前版本。

### 厂家启动文件实施后 SHA256

| 文件 | SHA256 |
|---|---|
| `src/bringup/launch/bringup.launch.py` | `c9fa405d194b09cc24226471d77052999b1ab47dfb9d2fa0b8f8c24935dcccad` |
| `src/slam/launch/slam.launch.py` | `d19d88fdeae1d3db028ab15bdc8ee12dfd8255604756c727f358720efdd7956b` |
| `src/slam/launch/include/robot.launch.py` | `877b65f340a471891f0ff59a1379b25efb491c00e90c728a5dcfdc2b9d72bb5f` |
| `src/navigation/launch/navigation.launch.py` | `293608c6d07a15d702d16a45e4edcd006c9a614971ff3282ae42e2fbe3b06632` |

## 保留的失败证据与异常

1. 第一版手写补丁格式错误，`git apply --check` 以 128 拒绝，实车文件未改变；之后从实车备份原文件生成补丁并通过检查。
2. ROS 清单占位邮箱 `@local` 不合法，第一次被识别为普通 Python 包；已改为合法占位邮箱并重新构建为 `ros.ament_python`。
3. 第一次隔离限幅取样过早，只读到启动零速度；延长输入后复测得到正确限幅值。
4. 厂家 `navigation` 全包构建仍会索取历史文件 `config/nav2_params_ack copy.yaml`。该文件不在源码且无代码引用；旧安装副本与旧 `nav2_params_ack.yaml` 完全相同，已备份并隔离而未删除。问题仍来自厂家旧构建元数据。本次需要的安装版 `navigation.launch.py` 已成功更新、哈希匹配并通过语法检查，但不得把厂家全包构建宣称为通过。
5. 第三次建图尝试在 SLAM 回环时出现 `0.052 m / 0.359 rad` 地图坐标修正，旧保护逻辑安全中止并停车；候选地图保存在 `/home/ubuntu/raicom_backups/20260821T1316_mapping_attempt3`。运动跟踪随后改用修复后的原始里程计，完整复测通过。
6. 首次看门狗探针以 50 Hz 灌入深度 10 的队列，观察端记录 `477.9 ms`；第二版探针因未持有订阅对象而未收到零速回调。这两次均判失败并保留，修正为深度 1 且持有订阅对象后连续通过。
7. 看门狗输出临时提高到 50 Hz 后，Python 单线程定时回调挤占输入回调，导致一次后退只累计 `0.137 m` 后超时停车。恢复 20 Hz 后，断控为 `112.6 ms`，后退 `0.202 m` 完整通过。
8. 第一版线性标定补丁 hunk 行数错误，`git apply --check` 以 128 拒绝且实车参数未改变；修正后从原始 `0.94` 备份重放，最终 `0.770` 补丁干净应用。
9. 首次从非交互 shell 启动家庭 Nav2 时，厂家 `navigation_base.launch.py` 直接读取未设置的 `need_compile`，启动以该键错误失败，所有 Nav2 组件随即退出且底盘保持零速；家庭导航入口随后显式设置该环境项，不再依赖终端配置。
10. 载入厂家现有 TEB 工作空间后，Nav2 在生命周期激活阶段仍无法建立完整 TF。曾将组合容器临时切换为 ROS2 自带的 `component_container_mt` 进行隔离验证；现象未消失，确认该假设错误，随后从备份恢复 `component_container_isolated`，源码与安装文件哈希均恢复为 `65c1c91b...bd90`。
11. 继续检查发现，新启动的订阅者无法取得开机时发布的 `/tf_static` 历史，导致 `lidar_frame` 不存在，AMCL 丢弃全部扫描。未重启基础 bringup，只单独重启原 `robot_state_publisher` 进程并沿用原 URDF 与参数文件；随后新订阅者可立即取得 `base_footprint→lidar_frame`，实测静态平移约 `(0.073, 0, 0.093) m`。
12. 一度以 `root` 用户恢复传感器与 Nav2，而厂家基础节点和 systemd 看门狗使用 `ubuntu`。Fast DDS 共享内存权限导致两组进程可以发现接口却收不到实时数据。相关测试进程已停止，传感、TF、里程计和导航验证统一改为容器内 `ubuntu` 用户运行；容器未创建、删除或重建。
13. 第一次实车导航到 `(-0.78, 0.10)` 时，控制器 10 秒内报告无进展，BT 随后反复执行旋转恢复并最终 `ABORTED`。失败现场中 `/odom_raw` 累积平移约 `8 cm`、航向等效约 `+40°`，EKF `/odom` 航向约 `-56°`，二者符号相反；AMCL 同时跳到约 `(0.66, 0.49, -79°)` 并停止刷新。该失败证据保存在 `/home/ubuntu/raicom_backups/20260821T1740_nav_guard_route/nav_run_01.txt`，在修复并隔离验证 EKF/IMU 航向符号前禁止再次发送导航目标。
14. 厂家 EKF 的完整运行命令、切换前进程状态和 RAICOM 源码已备份到 `/home/ubuntu/raicom_backups/20260821T1842_odom_bridge`。当前直通桥尚未完成实体左右转符号复核，仍禁止直接重试完整 Nav2 目标；如复核失败，停止桥并按备份中的 `runtime_before.txt` 恢复 EKF。

## 未验收

- 连续 5 次 `home_pick_station` 定点导航；
- ID10 两指夹爪小幅开合 10 次；
- Aurora930 单方块定位和抓取；
- SCI/YOLO、比赛地图、比赛坐标与完整比赛状态机。
