# 新版深度抓取接入场地工作台

> 后续已获现场授权并通过实际工作台按钮执行一次：默认上电回位、夹取动作与保持读回通过；本次夹稳尚无现场确认，120秒总超时后取消，没有记成功。最新状态见[单次实体试验记录](DEPTH_WORKBENCH_PHYSICAL_TRIAL_20260922.md)，下文“未实物复验”描述的是安装完成时的边界。

已安装版本：`rgbd-upper250-hold-20260922`。这次更新工作台对新版抓取的启动、阶段显示、停止与结果接收；原生抓取算法和实测参数未改。用户已确认 trial_1790080023796179196 的实物夹取效果成功。本次工作台接入完成软件验证，尚未通过工作台按钮复验实物。

## 现在如何使用

1. 打开场地工作台。沿用已保存的默认自动上电与启动姿态；启动时机械臂会运动，先保证动作范围净空。
2. 进入“深度抓取”，停车、空爪并摆好前方蓝色方块，点击“抓取前方蓝色方块”。按钮会准备接口并启动一次完整抓取；不用另外运行原生 launch。
3. 自动执行观察姿态 → 寻找上层有效蓝块 → 对准 → 前伸夹取 → 指定姿态保持。画面默认折叠，过程仍持续显示，可自行展开画面。
4. 到保持姿态后自动停止本次抓取程序并交还控制权，夹爪继续保持夹紧。界面显示本次五关节保持读回和夹爪读回。
5. 已标定自动夹稳观察端时优先接收自动反馈；没有有效自动证据则显示人工兜底。点击当前件“确认蓝色方块已夹稳”才能报告夹稳成功。物体取走后确认空爪，再开始下一次；空爪确认本身不发送松爪指令。

本次安装时没有运行工作台或启动实体抓取，也没有重启用户程序。下次打开工作台使用新版文件。若已有旧工作台，应先结束当前动作并关闭，再重新打开。

## 固定参数与版本绑定

原生生产包：`/home/ubuntu/ros2_ws/src/rgbd_grasp`。工作台仍调用该包的 `rgbd_grasp.launch.py`，只关闭重复的 servo controller、hardware、description 启动，不覆盖成功参数：

| 设置 | 当前值 |
| --- | --- |
| 抓取观察姿态，ID1/2/3/4/5/10 | 450 / 800 / 50 / 250 / 860 / 390 |
| 多目标选择 | `min_v`，最上方有效蓝块 |
| 抓取高度补偿 | 0.015 m |
| 抓取过程 ID4 补偿 | +60 pulse |
| 抓取后保持姿态，ID1～5 | 450 / 500 / 250 / 400 / 860 |
| 保持过程 ID10 | 保留原夹紧指令，不重新张爪 |
| 原生到位容差 | 沿用 30 pulse；没有改成工作台启动姿态的容差 |

工作台启动姿态和抓取观察姿态是两个既有阶段；本次保留前者，不用抓取姿态替换它。旧发布版 `rgbd-original-side-20260922` 的 ZIP 和本地旧 `modules/rgbd_grasp` 保留，不能覆盖目前生产包。

## 导航和比赛接口

原 `/raicom/depth_pick/request`、`/raicom/depth_pick/status`、预览话题及 `raicom.depth_pick/v1` schema 不变。仍要求当前 `backend_id/client_id/request_id`、有效时间戳、唯一发布者和停止后控制权交还。比赛页复用同一个抓取后端。

状态和本次 `result.json` 增加 `motion_result`：

```json
{
  "selection": {
    "target_selection": "min_v",
    "grasp_height_offset_m": 0.015,
    "grasp_id4_offset_pulse": 60
  },
  "hold": {
    "goal": [450, 500, 250, 400, 860],
    "measured": {"1": 451, "2": 476, "3": 238, "4": 395, "5": 859},
    "gripper_readback": 104,
    "hold_pose_reached": true,
    "gripper_command_preserved": true,
    "physical_holding_confirmed": false
  },
  "validation_error": "",
  "physical_holding_confirmed": false
}
```

上例读回摘自用户确认的成功试次，不是下一次抓取的预置结果。实际值来自当前原生状态。`motion_result` 只证明抓取动作及保持读回；原生程序明确没有独立的实物夹稳判断。导航仍须 `state=succeeded`、`holding=true`、`arm_released=true` 和当前请求有效，自动或人工确认来源另见 `holding_confirmation`。不允许仅看到 `auto_grasp_complete`、ID10 大于 60 或到保持姿态就直接运输。

原生 `stopped` 会终结当前请求；取消后的完成回报无效。重复、迟到、缺少保持证据、舵机读回超出原生容差的完成回报不能升级为成功。此次没有新增 A/B 分类或放置落位感知；整轮自动比赛仍需现场标定与有效感知反馈，不能把这次单件成功当作整轮验收。

## 验证与安装证据

- Pi 原生 ROS 检查：99 项通过。包含新请求等待期间不显示上一件读回、确认后正确显示夹稳状态。首次 3 项失败来自候选目录下的测试源码定位，修复测试夹具后全通过；失败记录保留。
- DDS126、localhost：真实新版原生程序＋真实工作台适配器＋模拟相机和舵机，完整抓取、停止回执、控制权交还、人工兜底与当前编号结果通过。实物动作 0。
- Windows 真正 Tk 六页签离线预览通过；图像可折叠，抓取过程、停止按钮保留可见。
- 已安装 5 个模块：`depth_pick_release.py`、`depth_pick_motion.py`、`depth_pick_runner.py`、`competition_depth_runner.py`、`field_tool_depth_ui.py`。实际 build 路径导入摘要与候选一致；99 个受保护文件摘要未变。
- 安装时 Pi：<ROBOT_OR_HOST_IP>，沿用原 SSH 主机密钥；boot `f9c39e7b-08c2-4eb6-bed5-1c3e4f279e90`。当前控制域无工作台/抓取任务、无机械臂命令发布者。
- 本地证据：`runtime_evidence/20260922_220243_depth65_integrate/`。Pi 证据：`/home/ubuntu/raicom_candidates/20260922_depth65/`。
- 原件备份：`/home/ubuntu/raicom_candidates/20260922_depth65/production_before_1790086966999791616.zip`。备份含旧 4 个文件和新文件原本不存在的清单。

回退仅用于排查本次接入，不回退已经成功的原生抓取。须先关闭工作台和抓取程序，核对生产文件仍等于 `deployment.json` 的新摘要，再从该备份恢复 4 个旧文件并移走新增 `depth_pick_motion.py`。旧适配绑定的旧原生版本与当前新版不匹配，因此单独回退后会拒绝抓取；不能删掉版本检查硬跑。继续工作请使用本次新版。
