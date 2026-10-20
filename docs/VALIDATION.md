# 发布验证记录

## 已执行的本机检查

日期：2026-10-20。环境：Windows、Python 3.13、NumPy 2.4.3、pytest 9.1.1、PyYAML 6.0.3。使用隔离的项目测试依赖，禁写 pyc 和 pytest cache，没有连接 ROS 图、机器人或执行器。

- 原始 `rgbd_grasp` 核心测试 10 项通过。
- 发布副本 `raicom_control` 的 17 个明确选定的纯逻辑测试文件共 137 项通过。
- `tools/offline-suite.json` 明确列出上述 18 个文件。它只覆盖可移植的无头子集，不代表所有历史测试通过。
- `tools/check_publication.py` 检查 Python 语法、JSON、四个包的许可证、Python console 入口、安装数据存在性、七个抓取固定版本摘要和已排除资源。有限密钥特征扫描不等于保证所有秘密或二进制内容都已检查。

最终发布副本的重跑结果另记录在本文件末尾；GitHub Actions 结果必须以上传后远端运行状态为准，不能从本地测试推定。

## 扩展离线测试发现的未解决问题

扩展审查结果为 226 通过、23 失败、10 个设置错误，另有 21 个 subtests 通过。失败测试保留在源码中，没有删除或改成恒通过。

| 测试 | 当前问题 |
|---|---|
| `test_nav2_restored_profile.py` | 排除的私有 approach 配置造成设置错误，兼容二进制也未分发 |
| `test_navigation_acceptance_logic.py` | 私有地图、路线与 approach 缺失；还有一项发布 Twist 的契约断言与现有代码不一致 |
| `test_field_tool_navigation_logic.py` | lease 发布次数及 `zero_hold`/`failed` 状态的期望不一致；已派发导航目标后，当前代码跳过 GUI 传感器检查并继续续租，将有效性委托给 Nav2，详见下文 |
| `test_field_tool_workspace.py` | 一项到达容差外的状态断言不一致 |
| `test_guard_speed_offline.py`、`test_guard_diagnostics86.py` | 测试绕过构造函数，fixture 缺少当前代码使用的 `nav_angular_accel` |
| `test_vision_adapters.py` | 订阅销毁数、YOLO 启动方式及 rclpy.executors mock 不一致 |
| `test_vision_preview.py` | 状态文本断言不一致 |
| `test_profile_fusion_standby.py` | 导入的 `deploy/profile_fusion_standby.py` 在原工作区中也不存在，未运行 |

没有把依赖 ROS/Tk 的测试作为纯 Python 测试执行。运行完整 pytest 可能遇到上述失败和缺失 ROS 依赖；应先按所需环境准备，再逐项修复测试或实现。

## 尚未验收

本次没有进行 Ubuntu/ROS 全包构建、Nav2 原生插件构建、真实相机取帧、手柄热插拔实测、Tk 桌面交互、导航全程或实物夹取。历史文档中的特定设备试验不能替代本发布副本的这些验收。

## 导航传感器有效性的实际边界

只读复现确认：目标派发并进入 ACTIVE 后，即使 GUI 的 `sensor_ready` 变为 false、`navigation_input_error` 返回 scan stale，当前分支仍保持 active、未取消目标并继续发布有效 lease。`field_tool_navigation_logic.py` 的 ACTIVE 分支有意将扫描/TF 有效性委托给 Nav2；续租心跳也不独立检查传感器。

这不是已经验证通过的“导航中丢传感器立即停车”。本次不更改机器人运行控制逻辑；实际停止行为仍需对 Nav2、TF/costmap 与底层命令超时进行隔离故障注入，再做现场验证。该已知边界保留在公开文档，不能把对应失败全部解释为测试过时。

## 发布打包修正

原 `raicom_control/setup.py` 注册了不存在的 `command_mux.py`。发布副本移除这一失效 console 入口；其他入口由检查脚本逐项核对。原工作区没有修改。

## 发布副本最终重跑

`tools/run_offline_tests.py` 合并运行 18 个文件：**147 passed**，没有连接 ROS 或硬件。打包检查通过：4 个 ROS 包、28 个有效 console 入口、7 个固定抓取摘要。首次整理时 GitHub CI 尚未执行；上传后的结果见下一节。ROS 全包构建和实机验收仍未执行。

## GitHub 首次发布验证

2026-10-20，公开仓库 `Parco3037/little-car-ros2` 的提交 `8b534110761cabfd777069d1ec818c337b49a385` 已完成 [GitHub Actions 无头检查](https://github.com/Parco3037/little-car-ros2/actions/runs/39431997030)：Ubuntu 22.04 / Python 3.10，**147 passed**，打包检查和七个固定抓取摘要全部通过。

已从公开仓库重新克隆，确认 291 个文件的完整 Git 文件树与发布副本一致，28 个 console 入口及七个固定摘要再次通过检查。已知历史测试失败仍按上表保留；这次绿色检查仅覆盖明确列出的离线子集，不证明 ROS 全包构建、导航故障注入或实机控制通过。
