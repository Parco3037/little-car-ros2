# RAICOM 历史缺陷登记与关闭门禁

> 阶段：`G0-HISTORICAL-BUG-CLOSURE`
> 建立时间：2026-09-03
> 性质：append-only 记录；本文件不是运行授权，也不替代 `PROJECT_HANDOVER_CURRENT.md`。
> 当前总判定：**NOT_PASS**。没有任何条目仅凭代码、静态检查、source/install 哈希或历史“在线”状态标记为 `CLOSED`。
> **Pi 最新只读覆盖（20:14）**：`NTPSynchronized=yes`，两个基础 systemd 服务 active，串口持有者仍唯一；基础 ROS 仍无 `command_mux`、`nav_cmd_bridge`、map、AMCL、Nav2、RF2O 或 matcher，`/scan≈9.99 Hz`、`/odom≈30 Hz`、`/controller/cmd_vel≈20 Hz` 且零速。Python 实际加载的 source/build 模块为当前 guard 哈希 `0c3a…`，但 `build/lib` 仍有旧 `9f447…`，Pi 尚无 `command_mux` 产物；BUG-010 继续保持 `OPEN`。defunct 短采样为 `4→4`，不替代 G0 长期验收。详细快照见 `runtime_evidence/g0_readonly_snapshot_20260903T201427+0800.md`。
> **Pi 新鲜覆盖（19:33）**：boot_id=`ab52c160-e5d8-42f8-b737-c48f28573636`，时间同步正常；基础服务和 `base_production.launch.py` 仍在线。`/scan`≈9.9 Hz、`/odom`≈29.9 Hz、`/controller/cmd_vel`≈20 Hz，零速 echo 全零；EKF 仅订阅 `/imu`、`/odom_raw`。未发现 `/odom_rf2o`、map/Nav2/bridge/fusion/matcher。该证据不替代 G0 的 3 次 cold boot、15 分钟和方向实测。
> **Pi 最新覆盖（19:53）**：boot_id 未变化；两个基础 systemd 服务 active，MentorPi、相机和 YOLO 容器在线。`/scan`≈10.0 Hz、`/odom`≈30.0 Hz、`/controller/cmd_vel`≈20.0 Hz，当前零速；`/odom_rf2o`、map、AMCL、Nav2、bridge、fusion、matcher 未运行。D 盘的最小 mux/bridge 候选未同步、未构建、未部署，不能改写 Pi 运行事实。
> **D 盘 Nav2 候选覆盖（19:53）**：`nav2_params_home_low_load.yaml` 的 AMCL/costmap scan 输入已改为唯一 `/scan`；它和 G1/G2 launch 仍只在 D 盘，未同步或运行。
> **范围纠偏（2026-09-03）**：17 项 preflight challenge、运行时 producer 和 encoder 协议取证撤出当前 G0；相关草稿已移入 `experimental/startup_preflight_17/`，不进入生产 package、launch 或 guard。G0 只验收基础运行链；编码器增强不再作为 G0 硬阻塞。当前最高优先级为 `STATIC_START_MAP_ALIGNMENT_FAILED`（约 median 0.20 m、P90 0.36 m，门槛 ≤0.10 m），它是 G1 定位前置阻塞。
> **Pi 实时覆盖（19:23）**：受控 base 切换后 `start_app_node.service=active`、`raicom_cmd_vel_guard.service=active`，`base_production.launch.py` 已运行。`/scan_raw`≈10 Hz、`/scan`≈10 Hz、`/odom_raw`≈49 Hz、`/odom`≈30 Hz、`/imu`≈49 Hz、`/tf`≈45 Hz、`/controller/cmd_vel`≈20 Hz，零速样本为全零；`/odom_rf2o` 为 Unknown topic。当前单一串口 holder 和基础 publisher 数量成立，但 G0 仍未完成 3 次 cold boot、15 分钟、僵尸趋势与方向实测；不把它记为 `CLOSED` 或 `PASS`。
> **18:31 历史 Pi 快照（已由19:33覆盖）**：`start_app_node.service=failed`，启动候选因旧 vendor base owner 未退出而 fail-closed；该状态不代表当前运行态。

## 关闭定义

单个 BUG 只有同时满足以下条件才可改为 `CLOSED`：

1. 有可复现触发条件、修复前后原始证据和自动化/静态回归。
2. D 盘 Git 回退点、Pi 时间戳备份、source/install/build/runtime 哈希完整。
3. 至少 3 次 cold boot；基础链持续 15 分钟；至少 10 条 Nav2 route（若该 BUG 涉及导航）。
4. 有 STOP、断开/错误路径和零速后置验证；涉及机械臂时还要有真实 action、实体到位/失败反馈证据。
5. 地图、PGM、route 的哈希与变更边界清晰，现场区域/朝向与启动激光对齐门禁通过。
6. 交接文档、原始证据索引、干净 Git 状态和下一步授权明确。

只有代码或静态审计通过：`FIXED_PENDING_REAL_TEST`；证据不足：`OPEN`；现状明确不满足：`NOT_PASS`。本轮不执行破坏性或非零测试。

## 总览

| 编号 | 缺陷主题 | 当前状态 | 本轮判定 |
|---|---|---|---|
| BUG-001 | 重复 launch/重复底盘实例 | OPEN | 当前仅一套 `base_production.launch.py`，但未完成 3 次 cold boot 回归 |
| BUG-002 | 多路 `/controller/cmd_vel` 绕过/争抢 | FIXED_PENDING_REAL_TEST | 当前 guard 单一发布者；guard 仅保留 motion_enable、超时、限幅和 stop 责任，cold boot/全链未闭环 |
| BUG-003 | MS200 scan 点数、量程、QoS/适配契约 | OPEN | 当前 `/scan_raw→filter→/scan` 约10 Hz且有样本；完整契约仍未验收 |
| BUG-004 | Fast DDS/跨容器发现不稳定 | FIXED_PENDING_REAL_TEST | 历史修复证据存在，未完成关闭定义的真实回归 |
| BUG-005 | PID1/zombie/残留进程清理不可靠 | OPEN | 受控 MentorPi 重启清掉旧残留；容器 PID1/zombie 三次启动趋势仍未验收 |
| BUG-006 | `/odom`/TF 所有权和自定义 fusion 冲突 | OPEN | 当前 `/odom` 单发布者；生产 EKF 不得引用 RF2O/自定义 fusion |
| BUG-006A | `/odom_raw` 为命令积分而非编码器反馈 | KNOWN_LIMITATION_PENDING_MOTION_TEST | 不作为 G0 基础链硬阻塞；须如实标注，不能称为轮速闭环 |
| ENHANCEMENT-STM32-ENCODER | STM32 真实编码器回传 | BACKLOG | 仅在标准 EKF+AMCL+Nav2 实车路线精度不足时启动 |
| BUG-007 | Nav2 CPU scheduling/低负载配置不足 | FIXED_PENDING_REAL_TEST | 配置和静态调整存在，当前 Nav2 未运行 |
| BUG-008 | STOP/bridge 永久锁存导致下一段不动或旧会话复用 | OPEN | D 盘已有最小长驻 bridge/mux/goal-context 候选但未部署、未实测；正式生产生命周期仍未实现 |
| BUG-009 | 服务位姿/工作位姿/碰撞模型混用 | OPEN | 启动区激光对齐仍失败，route 未改但未实车验收 |
| BUG-010 | D/Pi source/install/build/custom deploy 不一致 | OPEN | base 已运行且路径/哈希可核对；3 次部署/启动回归仍未闭环 |
| BUG-011 | Aurora/SCI/USB/低电压/视觉链耦合 | OPEN | 容器在线不等于相机帧、USB、供电或任务链通过 |
| BUG-012 | 横移不可靠，导航不能依赖 linear.y | OPEN | 高速横移实测出现约 13.3 deg 航向耦合；自动导航只允许原地旋转加直线 |

## BUG-001：重复 launch / 重复底盘实例

- 症状/触发：同时启动 vendor `robot.launch`/顶层 SLAM/Nav、APP 自启底盘或历史 factory/test 节点，造成同一串口、底盘、TF、scan 或速度出口重复。
- 本轮事实：19:23 当前实际运行一个 `base_production.launch.py`，Pi 进程中发现一个 `ros_robot_controller`、一个 `odom_publisher`、一个 MS200、一个 scan filter 和一个生产 EKF；当前没有 AMCL/Nav2/SLAM/bridge 叠加运行。切换调试期间曾因旧 owner 未退出而 fail-closed，随后通过一次受控 MentorPi 容器重启清理旧残留；不以此替代多次 cold boot 验收。
- 根因：`reference` 顶层 launch 会再次包含 `robot.launch.py`；历史架构允许不同入口混用。RAICOM `home_navigation.launch.py` 虽按不重复底盘设计，但 D 盘候选不能视为已运行。
- 当前状态：`OPEN`。基础运行态暂未重现重复实例，但生产 launch 尚未经过 3 次 cold boot 和完整栈验收。
- 必须证据：launch graph、进程树、每个 serial/话题/TF holder；Nav2 启动前确认 vendor bringup 隔离或明确复用。
- 回退/测试：保留当前 vendor bringup；任何生产切换先 Git 回退点+Pi 备份，失败只恢复启动层，不碰 STM32。

## BUG-002：多路 `/controller/cmd_vel` 绕过或争抢

- 症状/触发：teleop、Nav2 bridge、factory/test 或旧任务节点直接向底盘出口发布，造成多个 writer、旧命令残留或 guard 之外的非零速度。
- 本轮事实：`/controller/cmd_vel` 当前 publisher count 为 1，节点为 `/cmd_vel_guard`；guard 20 Hz 发布零速；`/cmd_vel`、`/raicom/teleop_cmd` 当前无活动 publisher。
- 根因/状态：唯一 guard 的基础约束已存在；旧 `home_mission` 直发 `/controller/cmd_vel` 的 D 盘代码路径已改为 `/raicom/nav_stop`，并由 G0 静态测试锁定。当前 guard 只接受四字段 `motion_enable` lease，同时负责 stop latch、输入超时归零、速度/加速度限幅和退出零速；完整“command mux→guard→driver”及真实 cold-boot/实车证据尚未闭环。`FIXED_PENDING_REAL_TEST`。
- 必须证据：cold boot 话题 graph、`ros2 topic info -v`、非零前零速抽样、STOP 后无残留 publisher、Nav2 bridge 只输出 mux 输入。
- 回退/测试：不改变 firmware；失败时停止候选 bridge/mission，保留 guard zero。

## BUG-003：MS200 scan 点数、量程、QoS 与适配契约

- 症状/触发：把 `/scan_raw`、`/scan`、`/scan_stable` 或 LD19/MS200 当成同一契约；点数变化、QoS 不兼容、错误 frame 或下游 2 m 截断被误判为雷达硬件故障。
- 本轮事实：当前链为 `/MS200→/scan_raw→scan_to_scan_filter_chain→/scan`，约 10 Hz；无 `/scan_stable`，无 RAICOM `scan_normalizer`。当前 `/scan` `frame_id=lidar_frame`、`range_max=2.0`。Pi MS200 实现与官方参考不同，Pi source/install exact。
- 根因：历史 source/reference 分支存在 MS200/LD19 与 filter return-list 语义差异，且生产 scan 话题命名未统一；2 m 下游限制与原始量程边界混淆。
- 当前状态：`OPEN`；尚未完成正式 scan contract、QoS、点数、frame、时间戳和 normalizer 单实例验收。
- 必须证据：连续原始样本、点数分布、QoS、frame、时间戳、下游参数和唯一适配器；不得以单帧或地图效果替代。
- 回退/测试：先保留当前单 filter；禁止叠加第二 normalizer，部署前建立回退点和 Pi 备份。

## BUG-004：Fast DDS / 跨容器发现不稳定

- 症状/触发：容器间节点或话题偶发不可见、启动顺序导致 graph 不完整、服务/动作等待超时。
- 历史证据：当前交接与历史恢复记录已有 Fast DDS、容器发现和启动顺序问题的修复/复核记录；本轮只读快照显示基础 ROS graph 可见，不能把一次可见当作稳定关闭。
- 根因状态：历史 root cause 与修复边界需要随具体证据索引核对；本轮不新增猜测。`FIXED_PENDING_REAL_TEST`。
- 必须证据：3 次 cold boot、容器 participant/node/topic/action graph、15 分钟稳定观测、目标启动顺序与超时原始日志。
- 回退/测试：只回退 Pi/Docker 启动层或 RAICOM launch；不重建/删除现有容器，不改 firmware。

## BUG-005：PID1、zombie 和残留进程清理不可靠

- 症状/触发：旧 bridge、定位、Nav2 或脚本进程残留；新会话读取旧 PID/TF/锁存，或者容器内 PID1 无法正确转发停止信号。
- 本轮事实：当前未见活动 AMCL/Nav2/SLAM/bridge；切换过程中复现过旧 vendor owner 残留和容器 PID1 不回收导致的 zombie，受控 `MentorPi` 重启后当前基础进程树已清理。当前 `start_app_node.service` 为 `Restart=no`，guard 是独立服务并有定向 ExecStop；仍未完成三次 cold boot 的 zombie 趋势证据。
- 根因状态：清理边界和 PID 复用风险仍存在；`OPEN`。
- 必须证据：每次 boot 新鲜进程树、容器 PID1、停止前后进程、无 zombie、无旧 ROS participant、无旧 latch/session/PID 引用。
- 回退/测试：清理只允许命中本次会话和明确进程；不能 broad `pkill` 触碰底盘/相机容器。

## BUG-006：`/odom`/TF 所有权和自定义 fusion 冲突

- 症状/触发：多个 `/odom` publisher、重复 `odom→base_link`、自定义 fusion/legacy odom 与 EKF 竞争；或者将 odom 原点误当 map pose。
- 本轮事实：19:23 `/odom` 只有生产 `/ekf_filter_node` 发布，`/odom_raw` 只有 `odom_publisher` 发布；`/tf` 有 EKF、IMU filter、robot state publisher；无 `/map`、AMCL、Nav2。当前 `odom_publisher` 用速度指令积分 `/odom_raw`，没有 encoder feedback。进一步静态核对确认 `MotorState` 只有 `id/rps` 命令字段，RRC 节点没有 motor feedback publisher，SDK 没有 MOTOR/encoder feedback parser。原厂 JGB-3865 规格书（SHA256 `d81b6e88d2068897113df2f28932d6531da2f9cfc0e6c34e5195a1e060c270fa`）描述 AB 相编码器、PPR=11，但未证明当前线束、STM32 回传或 Pi 解析链成立。当前 `/odom_rf2o` 为 Unknown topic，生产 EKF 不再订阅它。
- 根因：当前基础 odom 语义是命令积分；RF2O、自定义 fusion 和 scan matcher 不进入第一版生产 EKF。`OPEN`，但其中“缺少真实 encoder feedback”单独归入 `BUG-006A`，不再作为整个 G0 的硬阻塞。
- 配置补充：官方/Pi `controller/config/ekf.yaml` 还定义 `odom1: odom_rf2o`；当前 live 参数虽然 `odom1_config` 未设置，但无 publisher 的遗留订阅仍存在。G1 清理必须只改 Pi 运行层，不改只读官方快照。
- 必须证据：源码订阅/发布、live graph、TF frame authority、encoder raw feedback、EKF source selection、map→odom 单一 AMCL/SLAM owner；自定义 fusion 只能离线。
- 回退/测试：保持当前零速基础态；禁止用 `/odom=(0,0)` 或旧 TF/initialpose 授权导航。

## BUG-006A：COMMAND_ODOMETRY（已知限制）

- 当前事实：`/odom_raw` 由命令速度积分，不是 STM32 编码器反馈；当前不能描述为轮速闭环。
- 当前状态：`KNOWN_LIMITATION_PENDING_MOTION_TEST`。
- G0 级别：不阻塞 BASE_READY 基础链；仍须在配置、报告和运行证据中如实标为 command odometry。
- 后续验证：在获得当次现场授权后做低速前进、横移、旋转方向与 `/odom`/IMU 约定一致性测试；该测试不等同于 encoder feedback 验收。
- 禁止事项：不因本条修改、刷写或重新设计 STM32 固件、协议、PID 或麦轮运动学。

## ENHANCEMENT-STM32-ENCODER：真实编码器回传（后续增强）

- 当前状态：`BACKLOG`。
- 启动条件：只有标准 EKF + AMCL + Nav2 的实车路线测试完成后，仍不满足精度要求，才重新打开本项。
- 当前不做：不继续 encoder 协议取证，不修改或刷写 STM32，不改当前 `/odom_raw` 命名和语义。

## BUG-007：Nav2 CPU scheduling / 低负载配置不足

- 症状/触发：Nav2、SLAM、camera/vision 和 ROS executor 抢占 CPU，出现 missed deadline、TF timeout、DWB 无有效轨迹或实际速度不稳定。
- 本轮事实：D 盘存在 `nav2_params_home_low_load.yaml` 和 staging 低负载设计；当前 Nav2 没有运行，所以本轮没有 CPU/route 真实证据。
- 根因状态：历史已识别为调度与资源预算问题；当前修复只能记 `FIXED_PENDING_REAL_TEST`，不能记关闭。
- 必须证据：CPU/内存/温度/participant 监控、15 分钟基础链、至少 10 条 route、DWB/TF/scan deadline 和底盘速度反馈。
- 回退/测试：先按低负载配置运行只读/零速 preflight；任何失败回退 launch/config，保持 guard zero。

## BUG-008：STOP/bridge 永久锁存导致下一段不动或旧会话复用

- 症状/触发：第一段成功后验收器发终端 stop，bridge 永久锁存；未重启 bridge 即开始第二段，Nav2 仍有路径但底盘无位移。旧 bridge/session 也可能被误当作新授权。
- 本轮事实：当前没有 bridge 运行；D 盘已把候选 bridge 的停止/goal 状态改为长驻对象，使用 `boot_id/mission_id/goal_id/expires_at` 和显式清空，不再以重启清状态；但候选未部署、未验收，且当前尚无 goal-context producer。
- 根因纠偏：正式运行期间 bridge、command mux 和 guard 长期运行；段间只更新 `boot_id`、`mission_id`、`goal_id` 及消息 TTL，goal 结束清空当前 goal，不杀进程清状态。
- 当前状态：`OPEN`；不得把旧 `navigation_latched`/restart 方案写成已修复。
- 必须证据：长驻 bridge 的 fresh goal identity/TTL、每段结束清空 goal、STOP 后零速、旧消息拒绝、无旧 session 复用；多段真实 route 回归。
- 回退/测试：未部署候选仍留在 D 盘；部署前 Git 回退点+Pi 时间戳备份，失败恢复到上一运行包。

## BUG-009：服务位姿、工作位姿、碰撞模型混用

- 症状/触发：把启动区/识别区/夹取区/停车位的 pose 混为一个导航目标；把视觉槽位、服务位、机械臂工作位或永久 footprint padding 互相代替，导致地图对齐或碰撞判断错误。
- 本轮事实：保存地图、PGM、5/5 route 未修改；最新启动区静止激光对齐约中位 0.20 m、P90 约 0.36 m，仍未通过 ≤0.10 m 门禁；独立候选连续指向识别区接近位，不能当启动区初值。
- 根因状态：实体区域/车头朝向与保存地图观测冲突尚未解释；`OPEN`。
- 必须证据：真实区域/朝向确认、fresh map→base alignment、服务位姿与工作位姿分离、footprint/arm clearance 独立验证；不能放宽匹配域。
- 回退/测试：不重扫、不覆盖现有地图；学校场地变化时新建新时间戳地图/route，不混用旧资产。

## BUG-010：D/Pi source/install/build/custom deploy 不一致

- 症状/触发：D 盘改动被误当作 Pi 运行代码；source/install 看似一致但 Python import 落到 build；自定义 recovery/snapshot/bridge 文件缺失或内容不同。
- 本轮事实：RAICOM 核心 source/build/install/runtime 路径已按 `ros2 pkg prefix`、Python `__file__`、进程 cmdline 和文件哈希核对；实际 Python import 落到 `/home/ubuntu/ros2_ws/build/...`。20:14 进一步确认 Pi source 与实际加载的 guard build 模块同为当前 `0c3a…`，但 `build/lib` 仍有旧 `9f447…`，且 Pi 尚无 `command_mux` 产物，因此完整 source/build/install/runtime 闭环仍未完成。guard 层和 `base_production.launch.py` 的既有部署事实保留；D 盘 mux/bridge/setup/launch/service 候选未同步、未构建、未部署，不能混入运行代码。
- 根因：Pi colcon build 链接树是运行时入口，且自定义 base 切换尚未完成；仅比较 D/source/install 不足。`OPEN`。
- 必须证据：目标文件清单、A/B/C/build/runtime import path 与 SHA、安装入口/wrapper 语义、部署前后 Pi 备份；每次部署记录 Git 回退点。
- 回退/测试：部署前先备份和 hash seal；失败从 Pi 时间戳备份恢复，不用 `git reset --hard` 覆盖用户工作树。

## BUG-011：Aurora/SCI/USB/低电压/视觉链耦合

- 症状/触发：相机容器在线但无新帧，USB/串口设备漂移，供电原始值被误换算，Aurora 深度或 SCI 分类结果被错误当作导航/抓取授权。
- 本轮事实：`pi_cam_env`、`yolo_env` 当前容器在线；本轮没有取相机新鲜帧、没有跑 SCI/Aurora 任务、没有机械臂动作。RRC battery 只记录原始 UInt16 `8448`，没有换算。
- 根因状态：USB 设备、帧新鲜度、深度质量、供电单位、任务失败回退均未在本轮闭环；`OPEN`。
- 必须证据：新鲜相机帧及时间戳、相机 topic/USB holder、SCI 类别 `GEAR/T_BOLT/BLUE/UNKNOWN`、Aurora depth 质量、抓取 action 真实反馈、断电/失败 stow/stop。
- 回退/测试：视觉与机械臂保持独立门禁；视觉失败不得触发底盘或抓取，保持 STANDBY_ERROR/零速。

## G0 范围纠偏与四种 profile

### BASE_READY（当前 G0）

只检查：唯一 STM32 串口 holder、唯一 MS200 holder、正式 scan 新鲜且唯一、正式 odom 与 odom→base 新鲜且唯一、最终 `/controller/cmd_vel` 唯一 guard、连续零速度、新 DDS participant 可收到 scan/odom/速度心跳、不存在第二套 robot/slam/navigation 总 launch。相机、机械臂、电压单位和 encoder 协议不在 BASE_READY 阻塞条件内。

### LOCALIZATION_READY（G1）

在 BASE_READY 上增加真实 map publisher、AMCL active、唯一 map→odom、完整新鲜 TF、地图/route/initialpose 绑定一致，以及静止激光地图对齐通过。当前 `STATIC_START_MAP_ALIGNMENT_FAILED`：median 约 0.20 m、P90 约 0.36 m，高于 ≤0.10 m 门槛。

### NAV_READY（G2）

在 LOCALIZATION_READY 上增加 Nav2 lifecycle、costmap 新鲜、无历史 goal/stop latch/bridge session、Nav2 关键节点不单核绑定且不使用 nice=19。

### AUTO_READY（G3-G5）

最后才增加 SCI、YOLO、局部对位、机械臂、夹爪和固定抓取/放置姿态。

## 本轮执行边界

- 未改 STM32 firmware、协议、PID、运动学、地图、PGM、route。
- 未启动定位、Nav2、SLAM、bridge；未发导航目标、非零速度、机械臂或夹爪 action。
- 未进行物理拔线、实体舵机动作、视觉任务或低电压换算。
- 下次开机必须重新做新鲜 boot 审计；本文件的 PID、TF、boot_id、mission_id、goal_id、Screen Live 和现场安全确认均不得复用。

### 范围纠偏覆盖说明（2026-09-03）

上一条记录属于范围纠偏前的历史快照，不再代表当前生产设计：17 项 challenge/producer 已移入 `experimental/startup_preflight_17/`，不在生产 package/launch；生产 guard 改为四字段 `motion_enable` lease，静态测试只证明代码契约，不证明 Pi/cold-boot/实车通过。

## 本轮代码级回归证据

- `test/test_g0_code_boundaries.py`：范围纠偏及首版生产 base 候选收口后 11/11 通过。
- `test/test_nav_control_logic.py`：9/9 通过。
- `test/test_command_mux.py`：4/4 通过；只验证最小 mux 选择、超时、错误输入和导航优先级。
- `test/test_mission_logic.py`：4/4 通过。
- `test_scene_mission_logic.py` 因本机缺少 `pytest` 未加载；`test_nav2_restored_profile.py` 因本机缺少 `PyYAML` 未加载。工具缺失不计为通过。
- `home_mission.py` 已取消直接最终速度发布，改发 `/raicom/nav_stop`；D 盘新增的 mux、bridge goal-context、setup、launch 和 service 均未部署、未做实车验证；Pi 当前仍是 guard-only 运行态。
- `experimental/startup_preflight_17/` 仅保留17项 preflight/challenge 历史草稿；当前 `test_guard_logic.py` 7/7 验证四字段 `motion_enable`，17项代码未部署、未做 Pi/cold-boot/实车验证。

## 2026-09-09追加：当前启动流程反例与防回归（OPEN，非原G0收口重启）

用户最新目标包含定位基础、地图激光对齐/五区/导航、可重复启动、错误复盘；不再要求助手重建地图。当前运行事实与权限仍只见CURRENT；上方09-03快照、mux设计和旧关闭门槛不覆盖它。本轮没有部署或启动17项preflight、新mux、建图或导航。

| 追踪项 | 已验证事实 | 未完成的修复/验收 |
|---|---|---|
| BUG-001A 数量误判 | 两session用子串匹配，把10/11/19/100发布者或订阅者当1；D/Pi源码同SHA，Pi假数据复现 | 精确字段解析、重复/缺失字段拒绝、0/1/2/10/100回归 |
| BUG-002A 假零速 | .nan/.inf/garbage等经awk数值强转后被当零；本地/Pi隔离函数复现，无真实非零/NaN发布 | 有限/结构/值校验，解析失败拒绝；连续窗口与单样本分开验收 |
| BUG-005A 退出无上限 | 实际supervisor控制流在模拟子进程忽略TERM时40虚拟秒仍不退，500次terminate且0次kill | 有界信号升级/回收、真实受控取消与超时路径；不扩大进程清理范围 |
| BUG-001B 跨入口竞争 | 锁在desktop层，Windows直接入口绕过此锁；mapping/nav底层无共用锁 | 共享互斥及重复/跨入口并发的无硬件回归；尚未真实并发启动 |
| BUG-010A 检查不一致 | mapping host查串口唯一holder；nav host仅查设备/USB；保存分辨率正则允许0.021，覆盖检查只在开头 | 统一必要检查与保存竞态测试；不得用实际地图做覆盖实验 |
| BUG-006B IMU稳态拒绝 | 当前标定已应用，1453同步样本中1448被重力模阈值拒绝，零偏修正仅极少进入；仍有静止yaw漂移 | 单板适配/滤波/融合置信度修复与长窗复测，不盲调阈值冻结yaw |
| BUG-003A SLAM扫描门控 | 本机2.6.8外层shouldProcessScan只按XY距离，纯航向变化被正平移阈值筛掉 | 定向修复、静止/纯旋转/平移处理率与负载回归，未部署 |

现有9项入口字符串检查全过，但新增执行型反例只有22/50通过、28/50失败；**此处的通过率是未修脚本的审计结果，不是修复成功**。6项Pi假数据核查也全部误接受。禁止再用关键字存在、单样本0速度、单次打开窗口或D/Pi哈希一致替代端到端可靠性。

证据与可运行反例：[startup_contract_audit_20260909](../runtime_evidence/startup_contract_audit_20260909/README.md)。IMU实测另见 [imu_gate_readonly_20260909](../runtime_evidence/imu_gate_readonly_20260909/README.md)。当前未改生产脚本/参数、未动车；不得把这些新增诊断条目写成已修复。

## 2026-09-09追加：静止软件修复后状态（覆盖上一节未部署状态）

用户已允许静止软件修复；实际修改现有入口、没有新mux/17项preflight/硬件运动。

| 追踪项 | 当前结果 | 保留的验收边界 |
|---|---|---|
| BUG-001A | FIXED_DEPLOYED：精确字段且恰好一次；65个实际函数数量反例通过 | 不等于全部DDS故障场景已经验收 |
| BUG-002A | FIXED_DEPLOYED：严格Twist结构+Decimal有限精确零；24个数值反例通过 | 入口单样本与本轮独立120秒连续观测分开；没有单样本冒充持续零速 |
| BUG-005A | FIXED_DEPLOYED：TERM/KILL有界升级；Pi模拟父子均拒绝TERM，22.189秒全部回收 | 真实ROS全session退出/冷启动仍待验收；旧17zombie未清除；D状态进程不保证能杀死 |
| BUG-001B | FIXED_DEPLOYED：两个底层session共用锁；Pi私有fixture实际互斥通过 | 未并发启动两套真实ROS或运动会话 |
| BUG-010A | FIXED_DEPLOYED：nav补holder检查，保存精确2cm及排他发布；模拟写入竞态不覆盖通过 | 没有启动厂家arm制造真实串口竞争，也没对真实地图做覆盖实验 |
| BUG-006B | OPEN：私有新校零/固定零偏对照改善但仍漂移；正式配置未换 | 需本机多朝向标定数据及长窗、后续运动验证；不是参数调整即完成 |
| BUG-003A | OPEN：本机2.6.8外层扫描门控问题仍在 | 未改SLAM二进制/launch门控，未开展旋转/建图驾驶 |

最终120秒融合yaw净漂-0.73509°，趋势-0.26570°/min；不能因为某一窗口较小就称已修复。私有AB共4900条速度观测、最终静止观测2382条速度和2399条电机指令均为零；所有私有对照节点退出。详见[静止软件修复证据](../runtime_evidence/static_software_fix_20260909/README.md)。旧失败证据原样保留。

## 2026-09-09 17:55 用户边界覆盖：取消IMU标定及硬件改动

用户要求跳过IMU、今后不做硬件修改。BUG-006B的漂移事实保留，标为`KNOWN_UNRESOLVED_CALIBRATION_SKIPPED_BY_USER`，不是FIXED。上文“取得多朝向标定数据”不再是待用户执行的动作，也不作为SLAM/定位等全部软件工作的前置门槛。不关闭现有IMU、不写新标定矩阵、不要求拆板/接线/翻转整车；后续仅从既有硬件上的SLAM门控、TF/时间同步、激光匹配及定位软件继续排查。固件、协议、PID、电流/力矩和机械结构保持不变；效果不足须如实说明，不以伪造数据或降低停车保护绕过验收。唯一当前范围见CURRENT最上方。

## 2026-09-09 18:32 BUG-003A扫描航向门控修复（覆盖上文OPEN/未部署）

- **状态：FIXED_DEPLOYED_STATIC_CHECKED_PENDING_REAL_ROTATION，尚未CLOSED。** 用户只授权软件修复与静止验证；没有非零速度、导航或机械臂动作。
- 确定原因：SLAM 2.6.8外层 `shouldProcessScan` 只按XY过滤，正平移阈值把纯转向挡在Karto前。已改为平移或归一化航向满足原阈值即可进入；原时间、暂停、throttle和启动门控保留，不设平移阈值0。
- 本轮新增防回归事实：Karto 2.6.8的航向setter/ROS参数用弧度，getter却返回角度；首个候选实际库测试失败，未部署。最终补丁显式转换单位，12场景×两库共24组通过，含正反向、阈值、跨±π、静止600帧和保留门控。不要再次直接比较getter与弧度差。
- 完整隔离SLAM合成回放各150帧：原库1次、修补库24次匹配位姿，最大合成角误差约0.20°；仅私有DDS域87，与实车域0隔离。该结果不等于实体转向或定位精度通过。
- 正式部署只增加私有修补库并修改两份home_slam源码/安装入口；`/proc`已验证真实映射新库，SHA `1788d199…a83be`；原系统库SHA `d39e1bac…cb4ff`未变。D/Pi源码/安装launch同SHA `4f34b460…ead1f`。没有全量部署mux/fusion候选。
- 真实120秒：2400速度/2400电机指令全零，scan约9.958Hz固定500格，1165次采集时刻TF查询全部成功；SLAM约3.44%单核。退出后162/168条命令仍零，临时进程退出。启动10条queue-full仅在首个约0.9秒窗口，保留证据；未作逐消息TF writer归属验收。
- **BUG-006B仍未修复**：同窗odom净漂+0.644°、趋势+0.372°/min，IMU趋势+0.374°/min。变化小于0.02rad时没有新的匹配位姿，map/TF刷新不能算持续纠偏。保持取消IMU标定/硬件修改的用户边界，继续软件侧处理。
- 回退标签 `pre_slam_heading_gate_20260909_181103`；Pi `20260909_181103_slam_heading_gate/before.tgz`、D精确原件/修改前diff与原始运行包齐备。初次工具部署遇到目录属主限制、第一次观察器因旧rclpy单参数回调失败，均保留日志；前者在核对目标后用管理员权限仅安装选定文件，后者修正诊断工具后重测，不作为车辆故障或成功证据。
- 仍待验收：真实低速原地转向与平移扫描处理、移动匹配负载、定位稳定、已保存地图对齐、五区和导航。地图/route/彩色RViz资产指纹未变，本次未保存新图。

详细证据：[slam_heading_gate_20260909](../runtime_evidence/slam_heading_gate_20260909/README.md)。旧失败记录保留；本节不把全部历史BUG或整车目标标为通过。


## 2026-09-09 19:12 BUG-003A门槛静止对比/正式部署，BUG-006B仍OPEN

- BUG-003A保持 `FIXED_DEPLOYED_STATIC_CHECKED_PENDING_REAL_ROTATION`。上轮2.6.8航向门控库SHA `1788d199...a83be`不变；本轮仅将正式launch的正航向门槛`.02`调至`.005rad`，D/Pi源码/安装SHA `b96074aa...5722c`，实际/proc与运行参数已核验。不设距离0，不改EKF/IMU/固件/硬件或停车保护。
- 同一批真实扫描的私有180秒对比：首帧在内匹配1→3次，推导map/base航向净变化+0.5996°→+0.1997°；固定首5秒中值扫描参照的每帧P90残差中位数14.79→10.84mm，CPU2.45%/2.48%单核。两实例不发布TF、不保存地图；1791次scan时刻TF全成功，3700速度/3703电机指令均零。
- 不把新增扫描填图当精度：候选当前临时图残差的巨大下降部分来自栅格填充；冻结扫描参照也仍是同传感器一致性，不是独立物理真值。
- 正式180秒3581速度/3562电机命令全零、1763次扫描时间戳TF全成功，新增1次匹配，map/odom校正+0.3999°；raw odom净变化-0.1284°，近同时端点角度合成map/base约+0.2715°，仍有离散变化，**BUG-006B仍 `KNOWN_UNRESOLVED_CALIBRATION_SKIPPED_BY_USER`，不标FIXED，不恢复IMU标定前提**。
- 补齐TF作者证据：C++读取MessageInfo，基础25边/正式26边在各30秒窗均单writer/单parent且可归属；map/odom来自SLAM，odom/base来自EKF。另267次扫描时刻真实map/base查询全部成功，不把查询成功当定位稳定。
- 新/保留限制：正式启动9条queue-full只在首0.803秒；12秒临时图观察器有2次初期TF查询失败及184个图外点，未删除。正式退出launch0/SLAM信号-2、无残留；停止后187/181条命令全零，保护地图/点位/彩色配置哈希未变。
- 诊断工具trial_v1空数组参数abort、v2外部参数dump超时、v3请求未声明参数返回空列表，均拒绝继续并保留；v4核验关键已声明参数后再原样输入真实扫描，才计有效观察。首次正式测试因Pi未部署D盘test目录导入失败，SLAM未启动；隔离fixture引用实际launch后4/4通过，production_static_v2才计正式实跑。失败自己的core移入证据原件保留。
- 本轮回退tag `pre_slam_static_correction_20260909_184500`，D精确原件/完整dirty diff与Pi before.tgz齐备；仅选定home_slam生产文件被部署。未整体提交/重置用户工作树，未部署候选mux/fusion。
- 定位稳定、实体纯转向/平移、保存地图对齐、五区与导航仍NOT_ACCEPTED；后续在软件匹配响应/角度离散校正继续处理，实体动作仍需新现场授权。

原始数字与指纹：[slam_static_correction_20260909](../runtime_evidence/slam_static_correction_20260909/README.md)。当前停止状态只读CURRENT最上方。


## 2026-09-09 19:28 BUG-003A/006B精匹配步长候选已否决，未改生产参数

- 2.6.8真实源码确认fine_search_angle_offset作为精匹配步长。原生库重放2组真实静止记录（1791/1784帧），比较约0.2°/0.1°/0.05°，两轮12进程均返回0，6/6非耗时结果一致，ROS/mapper实际参数及加载库已读回。
- 第一组减小跳变但固定参照P90中位数10.84→12.89/12.30mm，航向跨度0.4100→0.4968/0.4468°；第二组无改善，故更细步长均REJECTED_NOT_DEPLOYED。没有以冻结模型/视觉更平滑凑通过。
- 保留正式heading门槛`.005rad`和fine步长`.00349rad`，launch SHA `b96074aa...5722c`、私有库`1788d199...a83be`不变；BUG-003A仍等待真实转向，BUG-006B仍未解决且IMU标定被用户跳过。
- 重放不发布扫描/TF/运动，不保存地图，域87/localhost隔离；缺失原始启动首帧历史，不能当完整实跑复现或独立精度。原生约7.03ms匹配耗时只代表静止小图，不代表移动吞吐。
- 19:25终检200速度/200电机命令零，base/guard/三容器保持运行，保护与生产11文件哈希未变，无测试残留；短窗odom仍漂，不记稳定通过。
- 下一步需要本次现场确认和实时监看，按已列出角速度≤0.05rad/s、每段≤4秒、段间停车的原地转向方案验证；尚未执行，不含平移/导航/机械臂/保存地图。

证据：[slam_matching_replay_20260909](../runtime_evidence/slam_matching_replay_20260909/README.md)。当前范围以CURRENT最新节为准。


## 2026-09-09 19:33 授权等待停止点

无新增BUG修复或实体验收。相同现场转向确认缺失连续出现于提出方案、19:31复核、本轮复核；没有运行试验可等待，完整目标进入blocked/等待用户，不重复重启试验。BUG-003A仍待实体转向，BUG-006B仍未解决，取消IMU标定/硬件修改边界保持。实际10秒零速、发布者和无试验残留证据见[authorization_wait_20260909_1933](../runtime_evidence/authorization_wait_20260909_1933/readonly_check.log)。恢复后必须重新核查状态并取得当前现场许可，旧确认不得复用。


## 2026-09-10 15:46 新boot复测与启动时钟缺陷

- BUG-003A：修补库与`.005rad`门槛在新boot实际加载；60秒7次匹配，1200速度/1200电机全零，568次scan时刻TF查询成功。仍`FIXED_DEPLOYED_STATIC_CHECKED_PENDING_REAL_ROTATION`，不得关闭。
- BUG-006B：同窗odom净漂+2.1577°，matched pose航向跨度1.1778°；继续`KNOWN_UNRESOLVED_CALIBRATION_SKIPPED_BY_USER`。不做IMU标定或硬件修改；不以map/odom端点抵消宣称稳定。
- 新发现 `STARTUP_CLOCK_UNSYNCHRONISED`：新boot Pi无默认路由，NTP packet=0，跨Windows相对时差约-56150.82s；Windows亦未NTP校准。当前未修复/未校时。内部scan/TF查找通过不替代跨主机校时与重启恢复验收。
- 新候选许可逻辑发现并修复“晚到画面先于控制tick，可能续上过期许可”；12项离线异常序列在D/Pi通过。仅工具目录模块，无ROS运动适配/生产接入，不算实车修复。
- 用户已同意有限转向范围，但新boot当前画面未建立，未发送非零指令；保护资产指纹未变。详见[新boot原始证据](../runtime_evidence/slam_yaw_trial_20260910_1526/README.md)。

- 校时前已进一步检查真实退出/启动范围：基础unit没有专用ExecStop，实际`/home/pi/robot_pi/start_node.sh`（SHA `5a070dce...dcbd6`）会按owner_regex清理ROS进程；base的`need_compile=False`使其实际包含source下的IMU/servo/odom子launch。真实`src/peripherals/launch/imu_filter.launch.py`仍配置`gyro_calib_samples=1000`，重启会重新触发既有陀螺零偏估计。这里只读确认，并未运行新标定、修改该参数或停用IMU；也没有将校时变成盲目基础重启。需要先完成有当前现场监看的受控退出/恢复核验，不能将重启视为无副作用、不能要求硬件标定作为前提。原始输出见本轮`clock_restart_host_scope.log`、`clock_actual_host_entry.log`和`clock_actual_children.log`。


## 2026-09-10 16:27 RViz软件渲染时序改善及诊断清理边界

- `RVIZ_SOFTWARE_RENDER_LOAD`：实际llvmpipe软件渲染，建图入口单行加入LP_NUM_THREADS=1、nice10、私有D-Bus。D/实际工具SHA `7cbed6db...761df`，Pi源码同一行修补SHA `cacedbb5...307c`。原90秒TF age P95 1.0143s；候选/正式命令60秒分别0.0663/0.1047s，扫描即时TF失败431/866→16/563与30/565。状态`DEPLOYED_STATIC_IMPROVED_NOT_FULLY_RESOLVED`，负载/窗口不同，仍有尖峰，不等于SLAM丢帧或定位误差率。
- `DIAGNOSTIC_VIEWER_CHILD_REAP`：诊断run2的dbus-launch提前退出后未回收，留下PID101111 Z/PPID1。修订子进程收养/组清理、私有D-Bus后，两项故障边界测试及run3/run4无新增残留。状态`FIXED_FOR_NEW_RUNS_LEGACY_ZOMBIE_REMAINS`；旧Z未回收，不以重启容器隐藏。run1显示授权失败core、shader告警原样保留。
- BUG-003A仍为航向门控已部署/静止验证、待实体纯转向；BUG-006B定位漂移未解决，run3实际map/base航向跨度1.0136°、run4匹配位姿跨度1.4694°。`STARTUP_CLOCK_UNSYNCHRONISED`仍开放；继续跳过IMU标定与硬件修改。
- 旧“无共享画面”阻塞已被实际VNC/RViz返回帧解除；当前缺的是跨日新boot后的现场状态，已问未答。有限yaw范围已同意，本轮无非零动作。最终143速度/200电机全零，70扫描时刻TF成功，11保护指纹不变。
- 所有失败和运行边界见[本轮证据](../runtime_evidence/slam_yaw_trial_20260910_1526/README.md)及CURRENT新顶部。静态通过不等于实车通过。


## 2026-09-10 17:07 有界yaw执行与低速无可见转动

- 新工具`yaw_once` v5已部署到独立raicom_tools目录并实际运行；0.05rad/s的0.4秒与2秒段有8/40条非零纯yaw及电机命令，段后20/20零指令。用户两次都未见明显车体转动且确认已停。BUG-003A仍`NOT_ACCEPTED_REAL_ROTATION`；1/2次新增匹配不能替代物理旋转。
- `YAW_LOW_SPEED_NO_VISIBLE_ROTATION`：0.05对应2.067rpm命令；实际SDK float32离线编码非零，未发现上位机整数置零。无MCU确认/轮速反馈，原因未定，未把它直接归咎硬件/固件/PID，未改这些边界。
- `YAW_PREFLIGHT_DISCOVERY_WINDOW`：4秒内29/39样本导致安全拒绝，修为最多10秒等足40/40。`YAW_VISUAL_WITNESS_TRANSPORT`：严格0.5秒保守年龄曾拒绝请求；优化挑战发出时机及编排，保持门槛，v5的2秒段取得4次新帧更新。PTY长JSON换行解析不足仍记录为客户端改进项，失败时不续许可。
- 一次Windows编排漏读结构化画面元数据，助手取消，零非零输出；所有7会话及5次运动前失败保留，不冒充全部一次通过。20项离线逻辑回归通过，不代表导航验收。
- 最终200速度/197电机全零、70次扫描TF成功、11保护SHA不变。反向未执行；0.10rad/s×1秒只作为需新授权的下一提案。详见[证据](../runtime_evidence/20260910_1631_yaw_executor/README.md)。


## 2026-09-10 17:27 0.10rad/s短段仍无可见转动，客户端解析修复已实际使用

- `YAW_LOW_SPEED_NO_VISIBLE_ROTATION`仍OPEN：0.10rad/s实际约0.85248秒、17条纯yaw/电机命令；用户仍未见明显转动且确认停住。轮速命令约4.134rpm，未获得MCU确认/轮速反馈，未归因于硬件或PID；不以6次匹配冒充实体转向。
- `YAW_VISUAL_WITNESS_TRANSPORT`：首轮0.524秒保守年龄在启动前拒绝，零非零输出。适配器补一次真实新帧获取而不改时效门槛；第二轮启动及一次续验通过、正常截止。传输余量仍有限。
- `YAW_PTY_CHALLENGE_PARSE`：Windows完整nonce解析模块已修复、9项离线回归通过并在本轮实际载入使用；断裂nonce/重复nonce/损坏帧/未知写入结果拒绝续发。不能从此推断所有终端分包形态均可恢复。
- 显式正向0.10/最多1秒档已选文件部署v6，默认0.05档不扩大；D/Pi25项逻辑回归通过，进程实际模块路径/哈希已录。没有生产guard/SLAM参数/固件/IMU或硬件变更。
- 最终175速度/179电机零命令、70扫描TF成功、11保护SHA不变、无本轮活动残留。BUG-003A仍等待实体纯转向，BUG-006B定位漂移与启动时钟问题仍开放。[原始证据](../runtime_evidence/20260910_1720_yaw_010/README.md)。


## 2026-09-10 17:49 实体纯转向更新首证据与段间角度参考缺口

- BUG-003A：首段用户确认明显实体转动，odom x/y零，30扫描/22匹配、最大匹配间隔0.20685秒；状态更新为`ONE_POSITIVE_REAL_TURN_UPDATE_OBSERVED_FULL_ACCEPTANCE_PENDING`。不能再写完全没有实体转向证据，也不能冒充90°、反向或定位稳定已通过。
- `YAW_SEGMENT_ORIGIN_GAP`：v7下一段前没有核验是否仍接着上段停稳航向。两段间实测参考变化，v8新增显式expected-start-yaw和2°拒绝门槛；D/Pi36项离线及真实66条odom只读拒绝验证通过，未用v8发起新动作。用户“动了”含义未明，不先归因用户搬车或传感器故障。
- `YAW_VISUAL_WITNESS_TRANSPORT`：第二段运动中画面时效拒绝后正常停车，仅0.9821秒，不算15°成功。缓存刷新从120ms改为200ms的候选只调整真实新帧获取时机，.5秒Pi时效门槛不变；JS10项通过，尚未新运动验证。
- `MODEL_FRAME_FORWARD_FORMAT`：首段用了不支持的low detail导致模型中途图像省略；实际返回帧/元数据和用户在场不等于模型看见中途画面。默认转发在第二段实际恢复，失败记录保留，不全算监看通过。
- 首段激光角度拟合16.10°与odom16.15°接近，但全点残差检查NOT_PASS；第二段局部4.30°与odom3.89°且P90 2.13cm通过短窗几何检查，跨段参考仍失败。BUG-006B定位漂移/启动时钟仍OPEN。当前90°目标未完成，182/200最终零命令、11保护SHA不变。[证据](../runtime_evidence/20260910_1734_yaw90/README.md)。


## 2026-09-10 18:46 补充证据：v10画面调度运行，90°约81.5°后参考漂移拒绝；NOT_PASS

本节append-only，不改历史条目为CLOSED。当前运行和授权以CURRENT顶部为准。

- 用户已明确“小车自己转了”仅指测试时转动、之后停车；段04/05后实体估计40°/52°，段08后确认尚差一点到四分之一圈。累计分段odom81.4868°只为近似角度，跨暂停全局odom仍不连续。90°未完成。
- v9记录原始整数header与接收时刻，v10将下一画面挑战请求时间按剩余有效期提前。精选工具已部署并D/Pi39项通过，v10段04/05/07/08实际非零运行。原0.5秒画面保守年龄、150ms许可、2°启动参考及4秒上限保持；未改生产guard/IMU/固件/硬件。Windows真实共享流目标4→8FPS，有成功运行和超时失败，未宣布长期可靠。
- 段06因VNC窗口最小化、无新画面被主动取消；段09因READY前航向参考变化>2°被拒绝。两次均0非零指令、20/20零尾窗、自己的SLAM/RViz退出。不得自动重设参考或删除失败重跑。段05达到4秒上限未达15°也保留失败。
- 门控修补库1788d199…83be实际加载。段03/04/05/07/08运动及后500ms扫描/匹配18/8、29/19、45/25、33/19、29/20；匹配header来自原scan，纯航向更新没有被平移门槛整段过滤。此证据只支持正向窗口更新，不关闭定位精度/导航问题。
- 原全幅几何：段02/03短窗PASS，段01/04/05/07/08 NOT_PASS；段08全点P90约.3522m，改变拟合比例最大角度跨度35.90°，不以trimmed最优结果冒充稳定配准。静止SLAM观察的即时TF失败9/8/0/17次保留，不混同等待后可用性或实际SLAM丢帧。
- 最后30秒581速度/600电机命令全零，270扫描时刻TF成功；IMU净漂-4.6778°、odom/实际TF净漂-4.6644°，scan约9.969Hz、odom约30Hz。基础链持续到达仍漂移，不能靠显示刷新率解释或修好。11项保护文件哈希不变，无活动yaw/SLAM/RViz残留，两个服务和3既有容器未重启，相机保持。
- 回退tag1753/1812、D精确原件及Pi备份已留；原dirty未reset/stage/commit。证据：`runtime_evidence/20260910_1803_stop_check/README.md`、`yaw_series_summary.json`、`after_segment09_observation`、`close_manifest.json`；1753分析及1734段03–09原始数据一并收录。本补充属于已修改/已部署/已运行、有界停车已核验；90°、稳定定位/移动对齐、地图保存/五区导航未验收。


## 2026-09-10 19:28 静止扫描重复性、TF投递诊断与原生参数对照；定位仍NOT_PASS

- BUG-003A正向扫描门控的已部署/已运行状态保留；本轮未再运动。60秒真实SLAM记录598扫描/24匹配，1198/1198全零指令；纯静止不等于新的转向验收。
- `SCAN_STAMP_TF_IMMEDIATE_ONLY`：新增只读探针保留即时失败并量化非阻塞晚到。369扫描中367即时可用、2次≤32.167ms、0次200ms到期；旧观察器另有24次即时失败，窗口不同，不能全部算恢复。诊断已部署运行，生产TF逻辑未改。
- BUG-006B及定位纠偏仍OPEN：60秒首末全部有效点几何P90 2.841mm，但同窗odom -6.78583°；实际同扫描map/lidar航向跨度2.2370°。漂移并非本轮静止原始扫描大幅变化所必需；没有独立物理量角或人为遮挡确认，不推断硬件故障。
- 原生SLAM/Karto/Ceres 15次平滑/细角度+9次合法网格对照，均未满足预先定义的跨度减半门槛；最佳约0.9529°对基线1.1941°，未选用生产候选。5mm/100mm非法组合被库拒绝，保留失败；细网格耗时上升。回环开关6次、3组1080扫描非耗时输出完全相同，不支持把本次残余归因回环。不存在生产关闭回环的改动。
- 转向08固定首扫描参照全点残差大，场景新可见范围/参照覆盖尚未分清，不能筛掉异常点凑通过。下一步需逐扫描几何和匹配历史诊断，避免重复无依据参数遍历。
- 最后10秒182速度/200电机零命令、70次扫描TF成功；guard/基础链、3容器、串口独占和11保护文件SHA保持。首次诊断路径脚本模块名错误、构建TBB漏链接均保留失败日志，并仅修补诊断工具后通过。全程无新的非零运动，90°仍约81.5°未完成。
- 回退tag1852、D精确原件和Pi before.tgz齐备；用户dirty未重置/提交；[运行证据](../runtime_evidence/20260910_1852_localization/README.md)及close_manifest记录精选文件、实际库与负面结果。状态是诊断修改/部署/运行完成、定位NOT_PASS，不关闭IMU漂移/时钟/导航问题；不恢复IMU标定或硬件改造前提。


## 2026-09-10 20:12 扫描匹配分阶段诊断与三项候选未通过；人工room会话已观察退出，保护资产不变

- BUG-003A已部署正向扫描更新证据保留，本轮未再运动；生产私有库1788d199…83be在用户人工SLAM进程928178实际映射中确认，后来会话退出。
- BUG-006B仍OPEN：原生分阶段回放1080扫描旧结果全等，solver Compute为0、匹配到图处理yaw差0；本批残余在逐帧扫描匹配已经出现。点对点/点对线/固定首次参考三项静止跨度0.9341/1.1326/1.2825°，均未达到基线1.1941°减半，全部未选用。失败/全点残差/退化和候选记录完整保留，没有生产参数变化。
- `STATIC_REFERENCE_FIT_GAP`：同源静止单帧多初值+首15帧中位参照拟合跨度0.0670°，支持继续拆分局部极值与参考构造影响；不是独立物理真值或已部署修复。不能用trimmed拟合隐去全有效点残差。
- `SESSION_CLOSE_PRECONDITION_STALE`：旧终检要求没有map/teleop/许可，而用户刚手动开启room会话，断言真实失败。未为通过测试停止用户会话；保留其199/199零指令原件，另进行不控制会话的状态取证。后来观察全会话退出；缺失退出方式证据，不记完整有界退出PASS。人工入口日志记录机械臂rest实际动作及读回，不能声称整个现场无动作；助手未发起。
- 新末尾199速度/199电机指令全零，70次扫描时刻TF成功，odom仍漂+0.2260°；11项保护SHA和独立guard、基础链、3容器启动时间不变。room新图未找到保存产物；旧81.5°不作人工会话后的当前朝向。
- D/Pi精确备份与tag1934齐备；独立诊断已部署/运行，生产SLAM/IMU/硬件/相机/网络未变。详见[本轮原始证据](../runtime_evidence/20260910_1934_scan_geometry/README.md)；最终上传只以close_manifest/close_upload为准。


## 2026-09-10 Aurora RViz插入项与romm积压新增证据

- 原人工 romm 会话在20:12交接之后发生，用户确认手动驾驶，之后确认退出且建图仍失败。原始日志974次队列告警、最大369帧、563次 past-TF 错误；10秒只读观察139条非零速度/电机、SLAM约95%单核、pose最大陈旧9.43秒，后续日志出现约49秒陈旧扫描。旧的81.5°朝向/旧PID均不可沿用；助手本阶段未发送非零动作。
- 2016的四组独立扫描拟合实验均未达到预先规定的静止航向跨度减半；最佳分别0.9328/0.9242/0.8907/0.9205°，原同窗基线1.1941°。均未选用/接入生产。原0.067°多初值诊断带有有利初始中心，不能当成可部署性能或真实量角证据。详情保留在 `runtime_evidence/20260910_2016_match_factors`；主线转向现场已证实的排队缺陷。
- 相机检查期间发现用户手动开启厂家 navigation.launch.py，原base已退出137，guard自动重启1次。用户明确授权结束该导航并恢复基础链。仅结束核实的该导航进程树，再启动既有start_app_node.service；其Aurora子进程随授权导航退出，pi_cam_env/yolo_env未停止。恢复后base/guard active，host MainPID1201627/1175105，guard NRestarts1；同boot 040d1883-6f7a-4c04-99c2-51ba05587d6a。旧2142/2143已不是当前运行PID。
- 恢复后10秒181速度/181电机全零，scan约9.969Hz、odom约30.011Hz、70次扫描时刻TF成功；速度出口/scan/odom各唯一发布者，map/teleop/许可发布者为空。新base建立了新的odom起点，不是地图初始位姿。保护11项文件SHA仍不变。
- Aurora已修改且精选部署：第三方包source/install viewer930_launch.py与aurora930-ros2.rviz，宿主 `/home/pi/raicom_aurora_viewer.sh`。补齐两个工作区环境，局部显示环境、独立DBus/单渲染线程，3m视距与较小窗口；aurora930_launch.py与相机二进制未改。原exit -6缺当时Qt stderr，未把具体崩溃根因写成确定事实。
- 相机独立有界测试真实收到深度43、红外121、彩色96、点云67条；四类各唯一发布者，RViz实际订阅，实际exe/argv/env/hash留存。用户视觉确认通过。自己的相机测试退出0，RViz测试也已结束，无活动残留；初次复现和终止试验留下惰性僵尸，未重启容器清理。布局/回收的后续候选未部署，转入证据的undeployed_followups，部署D副本已恢复一致。

下一步已授权的软件工作：使用同一2.6.8私有common库，保留shouldProcessScan航向/时间/暂停规则，取消同步类的额外无界队列，在匹配前拒绝>0.5秒旧扫描或>0.05秒未来扫描。先DDS87+localhost下对照200ms人工处理延迟，再仅部署选定入口和home_slam source/install启动文件，做共用锁/零速前后检的静止验证。静态通过不代表人工移动建图、定位纠偏、五区采点或导航验收。继续不做IMU标定或硬件修改。

队列修复前tag `pre_20260910_2040_online_queue`，D/Pi精确备份已建立，Pi before.tgz SHA 580542945327a40ebca5d3b0b882dd1021e226a39d29de94673c1a6b52b33bde。相机tag `pre_20260910_2049_aurora_viewer`，Pi before.tgz SHA ec669961f0a1cb2d4d1a6f9149aeb9b89729c4a0c52c6c500e52a0f317fdc5c8；[相机说明](../deploy/aurora_viewer/README.md)、[相机证据](../runtime_evidence/20260910_2049_aurora_viewer/)、[队列计划](../runtime_evidence/20260910_2040_online_queue/PLAN.md)。下方电量附加记录保留原样。



## 2026-09-10 21:42 有来源的在线队列修复与输入/导航缺口

- `SLAM_SYNC_UNBOUNDED_QUEUE`：已修改、精选部署、实际加载v3。DDS87隔离200ms延迟对照由123帧/17.491秒积压降为0额外FIFO/最大0.375秒结果年龄，9项通过。v1 DDS深度5失败保留；v2 KeepLast1通过；v3记录actual QoS depth1、attempted/matched及修复重复SIGINT清理。真实移动地图质量仍未验收。
- `STATIC_TRIAL_ZERO_IS_NOT_STILL`：工具observation_complete只检完成、零指令和pose年龄，不能作为物理静止或精度验收。v3预检185°、主窗IMU/odom126°且有效光束首末P90差0.979m；来源未明，用户搬动问题待答。保持原始文件和数值，不改成PASS、不归因硬件。v2的1条pose也不足以证明稳定。
- BUG-006A/006B：新只读30秒有效扫描首末差P90约2mm，但IMU/odom净变0.578/0.580°；1459条滤波后yaw协方差0，实际Humble源码默认orientation_stddev0；原始/校正IMU值0.01。odom实际加载源码从cmd_vel积分，停车x/yaw速度协方差1e-9。输入不确定度和独立观测不足仍OPEN，不通过随意填大协方差或停用IMU凑验收。
- BUG-002补充 `LEGACY_CMD_VEL_DIRECT_SUBSCRIPTION`：实际/cmd_vel当前无发布者但有odom_publisher订阅，源码app_cmd_vel_callback直接调用控制回调；/controller/cmd_vel唯一guard发布者不等于全路径受保护。未发旁路命令，未启动导航；必须在导航激活前隔离旧入口并验证guard独占。导航配置已为OmniMotionModel，不能照抄差速vy=0。
- 新输入窗口581/581全零、270次扫描时刻TF成功；base/guard与三既有容器保持，旧地图/route/彩色RViz/EKF/guard SHA保持，仅明确选定的SLAM入口与exe变化。未做IMU标定/硬件/固件/PID/相机变更。
- 方案以SLAM Toolbox #807/#499/PR545及2.6.8源码、Nav2/robot_localization/imu_tools官方资料、Lidarbot/RF2O项目逐项核对；用户部分外链访问失败未冒称读完。详见[来源与修复报告](SLAM_SOURCE_BASED_REPAIR_20260910.md)和[新证据](../runtime_evidence/20260910_2127_source_audit/README.md)。



## 2026-09-11 02:35 RF2O来源核对及离线准入：NOT_PASS，未接管定位

- `RF2O_IDENTICAL_SCAN_NONFINITE`：固定官方核心与厂家源码相同，5组准入0/5；静止68/68返回false并协方差NaN，其余各28/68失败，不能将exit0或最后有限行当成有效终点。源码aver_dt=0路径和官方#39为后续调查依据，未修改核心或生产融合。
- `REPLAY_CLOCK_TYPE_MISMATCH`：首版回放驱动Time(ns)与上游从header构造的Time类型不同，10次exit1；仅驱动改成从header构造，原整数时间不变，v2写完69行/组。该驱动修复不关闭上述算法失败。两版日志、二进制和SHA保留。
- `RF2O_SCAN_ANGLE_CENTER_AND_LATERAL`：官方核心以视场中心为零，ROS包装层twist.y固定0；本车0..2π和麦轮横移不能直接照搬。计算坐标修正使合成平移方向恢复，但准入仍失败，真实记录未继续回放、生产未接入。
- 新boot6b26d290…48e2下14项保护SHA保持，base/guard2135/2147且重启数0/0；171速度/188电机全零、70次scan时刻TF成功仅基础观察。BUG-006定位及旧/cmd_vel直达控制旁路继续OPEN；地图、点位和导航未验收。[全部证据](../runtime_evidence/20260911_0216_rf2o_replay/README.md)。


## 2026-09-11 03:04 旧速度旁路已封闭；新物理静止标签下定位仍OPEN

- BUG-002 `LEGACY_CMD_VEL_DIRECT_SUBSCRIPTION`：一处订阅修复已部署并实际加载，source813f0827…0d4b1。原隔离样本复现16条无许可非零模拟电机输出，补丁0；路由/三自由度/停车/超时11项通过。真实后窗399速度/398电机全零，旧cmd_vel底盘订阅消失。仅关闭这条已证实旁路，不宣称DDS安全隔离或正式导航路由全部验收。
- `SCOPED_CONTROLLER_RELOAD`：单节点切换保留base/guard/IMU/雷达等进程。一次性监督器容器167521管理167522并跟随base launch56；是当前运行依赖。3项生命周期隔离检查通过，未做真实整车重启验收。源补丁持久保留，其他控制/协方差参数未改。
- BUG-006 `STATIONARY_SLAM_YAW_VARIATION`：用户本次明确车已放稳；30秒里程计/IMU净yaw约1.295°，SLAM4条观察位姿净0.0659°但跨度0.7762°。XY<1.3mm，pose年龄<=0.1151秒且CPU3.27%，仍未通过预先固定的5条/0.5°条件。节点matched5含首帧，不能把观察条数混同匹配总数。5/7是短静止检查，不是定位或导航PASS。
- 同批scan到本轮栅格P90约1.6–1.8cm，只是内部一致性；未验证原保护地图或移动对齐。588/581全零，正常退出后199/199全零；原14项保护SHA保持。启动2条TF队列满提示保留，持续积压未复现。下一轮使用新静止证据，不补写旧v3场景大变的物理标签。[全部证据](../runtime_evidence/20260911_0240_cmd_route/README.md)。


## 2026-09-11 03:33 BUG-006 航向输入过度信任已复现；两项修正仍为隔离候选

- INTEGRATED_YAW_ZERO_COVARIANCE：实际IMU yaw协方差0，已安装EKF3.5.3下限1e-9；正式odom跟随其漂移。COMMAND_RATE_STOP_OVERSPECIFIED：零命令角速度也被标为1e-9，不代表实际停车测量。只删除积分yaw而保留假零速不是有效修复。
- 选定同时不融合imu yaw和命令yaw rate，保留IMU角速度及X/Y；未改标定、硬件、保护或任何协方差数值。候选49b5fe45…4a5a，仅工具目录运行；生产379053b8…1efa及正式EKF899未改。
- 合成6项通过，包括零命令下正反角速度响应和横移。三份真实数据分别改善0.951→0.121°、0.590→0.418°、1.675→0.544°；最后一份为用户新确认60秒静止。原生Karto同589帧连续yaw跨度2.375→0.390°，固定首5秒参照最差双向P90 15.94→10.88mm。候选仅2次匹配，完整静止/移动/导航仍OPEN。
- 加速度模长与既有滤波器稳态判据不匹配，真实debug话题60秒1/2911为true；没有因此重新标定或改判据。只积分角速度仍会漂移，输入选择修正不是独立全局航向观测。
- 最后199速度/200电机全零、70次scan时刻TF成功，15项保护SHA不变；所有本轮临时程序已退出，前轮必需控制监督器保留。[完整原始证据](../runtime_evidence/20260911_0310_yaw_causality/README.md)。

## 2026-09-11 03:59 BUG-006 / BUG-006A / BUG-010：EKF两项输入已正式修正，未关闭完整验收

原生源码与回放确认零协方差积分yaw及命令角速度造成不可信融合。只移除两项选择，保留真实IMU角速度、X/Y及所有协方差数值；独立source/install和D SHA均49b5fe45…e34a5a，已重载参数读回。真实坐标交接yaw差0.000425°、XY差0；切换约1.2秒TF间隙/12失败保留。新EKF393812及其生命周期监督器393037是当前base依赖，不能清理。未重启IMU/基础/容器。

用户新静止60秒：1200/1200命令全零、568次scan时刻TF成功。连续真实TF航向跨度0.36505°，XY最大轴跨度0.211mm；matched日志3、实际/pose1，不足以满足至少5条诊断，不能将单消息跨度0当精度PASS。仍有约−0.543°/分钟局部航向漂移，命令X/Y不是编码器。导航、长路线和cold boot未验收，不标CLOSED。实际边归属验证新EKF独占odom/base，临时SLAM正常退出；原保护地图/路线保持。

见[原始与回退证据](../runtime_evidence/20260911_0336_ekf_switch/README.md)。本记录只追加，不更改历史失败或用户dirty。


## 2026-09-11 05:07 清理归档暂停与 RViz 退出故障记录

- 存储清理已执行校验：265,224文件迁到D，Pi可用空间0.63 GB→8.09 GB；两层旧备份不再以Pi原路径为恢复入口，使用D归档INDEX.json。保留全部运行依赖/保护/地图/相机，用户要求当前暂停。
- RViz测试v4在退出时SIGSEGV，core曾使Pi满盘；现存core完整D备份核验后已移除。v5正常GUI关闭成功，但启动限制检查仍失败，尚未完成修正回归，不能将RViz/SLAM测试记CLOSED。前一阶段数据见runtime_evidence/20260911_0404_motion_ready。
- 最后15秒速度283/电机301全零，scan时刻TF120/0失败，15项生产/保护SHA不变；不代替定位、移动、五区或Nav2验收。无动作测试、无重启、无硬件或IMU改动。
- 后续工作只有用户明确恢复后继续，每次只取所需一套工具，用完流式迁D校验再清Pi。

- 本次收尾首次检查另发现保护地图目录为空，缺失原因未确定；目录mtime早于清理首个快照，删除清单没有保护源路径，但没有本阶段开始前的新内容快照。已按D原件指纹只补回缺失的YAML/PGM/data/posegraph四件套，现有route不变，最终15项通过。失败原文和恢复证据均保留，不能写成地图全程存在。


## 2026-09-11 15:40 RViz退出路径与启动竞态回归；地图缺失由用户澄清

- 用户明确地图为本人删除，修正上次“原因未定”；本轮不加载或再恢复旧图。用户恢复测试，并将存储规则改为“明确说迁回后再迁回”，v8/v9/v10留Pi。
- native RViz实际执行后再核实core0/0，启动限制竞态不再复现。scan发现前预检退出经严格有界样本等待修正。旧仅看残留进程的判断会漏报SIGSEGV，新5项回归覆盖真实v6=-11。
- v9实际取到同PID保存配置提示；按钮Qt动作名称Press而非click。v10精确Discard成功，RViz exit0且没有SIGINT/TERM/KILL，SLAM exit0，无自身残留。该关闭阻塞路径已修正验证；原生SIGSEGV内部根因仍未由堆栈证明，不泛化为所有RViz崩溃CLOSED。
- v10主窗TF200/0，scan最大年龄116.57ms，约7.15%单核；709/727指令零。真实pose样本仍少，主窗0；无新的物理静止标签或动作，定位/移动/五区/导航保持OPEN。13项非地图生产SHA未改。
- 最后boote037ecd9-529e-4383-9e83-e6aa80e4be63、服务active、223/241全零、TF90/0、磁盘约7.95GB。原失败均保留，v10原件仍Pi。见[运行证据与官方源码](../runtime_evidence/20260911_1516_v6_check/README.md)。


## 2026-09-11 16:11 受控左转后新增证据：SLAM持续匹配却抵消约40°航向

触发：用户新要求左45°，同boot、同生产EKF/SLAM/传感器，经当前现场和真实Screen Live许可执行。控制44.435°提前制动、约8.144秒，零尾窗及原生RViz/SLAM退出通过；实体角度回访待答。旧15°试验仅转3.19°、因500ms画面时效保护中断，历史结果不得覆盖。

FACT：82运动扫描TF全成功，年龄97–132ms；63个共同source-stamp pose/odom配对中，odom增44.449°、pose增4.057°，修正差逐步到-40.792°，最大pose年龄172ms。因此纯航向门控本次已实际通过，定位匹配仍NOT_PASS。原始result 595b736d…8234留Pi。全周ICP最好41.584°但全部点P90 226.2mm，原70mm标准不满足；不接入、不改阈值。尚未证明是哪一个Karto内部阶段造成偏差，不把该推测写成已修根因。

工具修复及验证：预热后捕获一次真实相对起点，保留2°开动前差值检查；独立45°策略上限0.15rad/s/20s，旧15°策略不放宽；poll严格无副作用；真实缓存帧150ms起取新图，仍以原500ms门限判定。56 Python+10 JS测试通过，v12 D/Pi/import指纹一致。RViz精确Press/Discard关闭再次exit0，无INT/TERM/KILL；原生全部崩溃原因仍未证明。

下一诊断：按2.6.8原生[Mapper::Process与图修正源码](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.6.8/lib/karto_sdk/src/Mapper.cpp)，在保留真实scan/header/odom的隔离回放中分别记录预测、顺序匹配、图链接和最终修正，不部署历史细配准候选或清障碍凑结果。现有数据支持这一排查顺序，不能认作已证明前端或Ceres故障。

最后基础链241/241全零、scan TF90/0、guard/base正常且无重启，未保存地图。按用户新要求保留Pi原始记录，等明确迁回；详见[本轮证据](../runtime_evidence/20260911_1516_v6_check/README.md)。


## 2026-09-11 16:49 TF链核对与前端匹配失真的阶段证据

TF实际连通、25条边单writer，运行模型/D/source/install SHA61f10a58…44b2；用户确认大致实物外参符合。临时原生产SLAM map链tf2_echo成功，主窗157/0，698/701零指令且退出0。辅助基础监听全轮2次TF失败未保存具体错误，不称全程0；随后独立12秒90/0，238/238全零。

相同left45输入direct/traced逐项等值，69次匹配累计前端顺序航向修正-41.992°、图修正0、Ceres Compute0。本轮故障主要发生在前端相关匹配。buffer10无解；smear.03改善转向但旧独立静止.390→.990°退化；进一步空间grid.005静止无改善。候选全未部署；非法smear.1/grid.005原生报范围错误留存，不修改SDK限制。真实近处障碍由用户确认，未删除或过滤凑结果。原全点几何残差失败保留。

工具适配故障：Humble rclpy订阅回调不接受第二个MessageInfo参数，首版只读探针TypeError退出；改成单参数只读收集，并用原生rclcpp MessageInfo实际归属DDS writer。失败日志保留。没有为取GID改ROS库。

完整来源、限制、未部署状态和新增桌面点位/导航需求见[运行证据](../runtime_evidence/20260911_1613_left45_replay/README.md)、[需求约定](DESKTOP_MAP_POINTS_NAV_REQUIREMENTS.md)。下一步待当前放稳确认录制60秒，不复用旧动作许可，不恢复用户删除的地图。


## 2026-09-11 17:05 已复现：旧angle filter的3m无效占位干扰Karto前端匹配

官方[laser_filters PR #202](https://github.com/ros-perception/laser_filters/pull/202)直接报告对AMCL/slam_toolbox的影响；本机实际third_party plugin为2.0.7、无replace_with_nan支持，加载库SHA92b26c42…f4fd。79对实时scan证明索引150–338共189束被标3m（最大量程2m），区外字节不变；Karto2.6.8未滤波匹配路径未排除这些有限超量程值。

仅改无效3m为NaN，同left45378扫描/全部113,115有效回波/源时间戳/odom保持，69次匹配转角3.234→45.026°，odom45.226°；参数未调。这是输入语义缺陷的因果复现，不是去除真实近障碍。旧静止589扫描仍失败（连续yaw.390→.942°，全点P90 10.884→13.649mm），运动全点P90约253mm未过；不得宣称SLAM导航已修好。

已构建SLAM消费者私有副本清理候选，未替换生产/未启动：online_range_candidate SHA92ee72e0…d462。原生float/double契约测试通过，新C++stage关闭/开启与对应旧控制/NaN对照除耗时外逐项全等。正式filter、公共scan、guard、EKF、home_slam和相机均未改。源代码和构建/回放证据见[修复说明](../deploy/slam_online_queue/RANGE_VALIDITY.md)、[完整证据](../runtime_evidence/20260911_1613_left45_replay/README.md)。

当前60秒放稳确认仍待答，record工具未运行；原始记录保留Pi、不迁回/删除。最后12秒199/240零指令、scan TF90/0，基础链未重启。


## 2026-09-11 23:35 全量审计与候选受控入口（未替换生产默认）

- `SLAM_INVALID_RANGE_SENTINEL` 仍为 `OPEN`，未切换生产默认 SLAM。Windows/Pi 宿主机/MentorPi 容器逐文件哈希审计完成，读取错误为 0；D 迁移基线 2398/2398 匹配。
- `home_slam.launch.py` 新增 `use_range_candidate`，默认 `false`；候选 binary 只在显式参数下加载。D/source/install SHA `1c49fec75af26cef398231deeaba96e6d46b0c7bfbf3ff3654f47ff2e762713c`。
- 候选隔离 DDS 87/localhost 实际启动：depth=1、正常 SIGINT 退出、无残留；默认生产入口隔离启动仍为 `54d3f6cb…11dc`。这不构成新 60 秒静止验收。
- 当前旧 60 秒数据 3 次匹配候选 0.5782°/基线 2.2213°，静止全点双向 P90 最大候选 20.5mm/基线 26.0mm；旧 45° 数据 69 次匹配候选 45.026°/基线 3.234°/odom 45.226°，但运动全点双向 P90 最大候选 252.7mm/基线 196.9mm，运动几何未通过。仍需当前 boot 静止数据及真实前进、后退、横移、反向转向分别验收。
- 回退 tag `pre_step1_range_semantics_20260911_232719`；Pi 备份 `/home/pi/raicom_backups/20260911_232719_step1_range_nan/container_before.tgz` SHA `36d64fdca4afa2e94a356def517ff6a13313474d890893921dd8d6bf5d4a0f86`。


## 2026-09-12 02:28 高速四方向验证：候选不切换，横移列为不可靠导航输入

- `SLAM_INVALID_RANGE_SENTINEL`：继续 `OPEN`。前进、后退、横移和 90 度左转的真实记录均在隔离 `stage_replay` 中对照生产与 NaN 候选。候选没有满足真实运动全点几何门槛；旋转虽恢复约 99.4 度航向跨度，但全点双向 P90 最大 `799.6 mm`。默认生产入口保持 `use_range_candidate=false`。
- `BUG-012`：高速横移 odom 航向变化 `-13.29 deg`，且用户已明确平移数据不准确。自动导航只允许原地旋转加直线前进/后退；建图人工 A/D 移动与自动导航策略分开记录。
- 用户确认左转实体约 90 度、现场净空、可急停；动作结论和原始哈希见 [第一步证据](../runtime_evidence/20260912_0228_highspeed_step1/README.md)。
- 未切换生产 SLAM、EKF、guard、scan 或 filter；未发送新的非零动作。


## 2026-09-12 02:52 场地工具软件已精选部署；实车 P0 尚未授权

- `FIELD_TOOL` 新增 Tkinter 三模式入口、TF 检查、时间戳地图保存、reference/approach 双点、地图绑定路线和逐点 Nav2 队列。
- 自动导航仅使用 straight 配置，`linear.y=0`，避免依赖实测不可靠的横移。桥接仍走既有 guard，工具不发布 Twist。
- 数据模型 5/5、导航状态机 3/3 通过；Pi 已构建并安装 executable。未启动或切换生产 SLAM/AMCL/Nav2，未发实车目标。
- P0 的当前地图单点导航仍为 `NOT_EXECUTED`，不因离线测试通过而改写为实车验收。
- 最终测试数为数据模型 6/6、导航状态机 4/4；隔离 GUI smoke 通过，桌面包装使用 subreaper 回收子进程。首次修正包装前 smoke 留下一个 `keyboard_input` defunct PID，容器未重启清理，不新增 active 进程。
