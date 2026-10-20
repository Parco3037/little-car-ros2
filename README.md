# Little Car ROS 2

面向 MentorPi / LanderPi 的 ROS 2 Humble 自研控制扩展：手柄接管、底盘指令门控、导航工作台，以及 Aurora930 深度相机的单次蓝块抓取。

本仓库公开 `competition_manual`、`raicom_control`、`rgbd_grasp` 三个功能层；`raicom_control/native/raicom_nav2_workspace` 是导航插件包。自研部分采用 [Apache-2.0](LICENSE)，第三方内容见 [来源与许可](docs/THIRD_PARTY.md)。厂商驱动、完整机器人工作空间和模型需要自行取得。

| 功能层 | 用途 |
|---|---|
| `competition_manual` | Xbox 热插拔恢复、回中后重新使能、手动与语音接管 |
| `raicom_control` | 速度门控、看门狗、定位与导航、视觉接口和现场工具 |
| `rgbd_grasp` | 同步 RGB-D 蓝块定位、坐标转换、IK/FK 预检、单次抓取并保持 |
| `raicom_nav2_workspace` | Nav2 控制器和机器人自身区域过滤插件，需单独发现和编译 |

## 安装

目标环境为 Ubuntu / ROS 2 Humble，机器人参考平台为 LanderPi Mecanum。Windows 上的离线检查不等于在机器人上构建通过。

先阅读 [外部依赖与资源准备](docs/DEPENDENCIES.md)，安装合法取得的厂商工作空间并加载其环境。在一个新工作空间内克隆本仓库，随后执行：

```bash
source /opt/ros/humble/setup.bash
source /path/to/vendor_ws/install/setup.bash
python3 -m pip install -r requirements-test.txt
colcon build --symlink-install --base-paths \
  src/competition_manual src/raicom_control src/rgbd_grasp \
  src/raicom_control/native/raicom_nav2_workspace
source install/setup.bash
```

需要 TF 可视化、语音或历史导航兼容层时，应先准备各自的本地资源。仓库不附带语音模型、反馈音频、实地地图、厂商生成 URDF 或预编译共享库。路径、标定和部署前提见 [配置与移植](docs/PORTABILITY.md)。

## 深度相机抓取

`rgbd_grasp.launch.py` 包含相机、蓝块识别、目标转换、运动学服务和一次抓取动作编排。默认选择最近深度组中的上方蓝块（`nearest`），不是通用物体检测，也不包含自动定点放置。

先在已有底层控制和舵机服务的环境中验证感知：

```bash
ros2 launch rgbd_grasp rgbd_grasp.launch.py \
  motion_enabled:=false auto_prepare:=false auto_grasp:=false
```

相机已运行时额外设置 `start_camera:=false`。当前入口即使关闭相机启动，也会在生成 launch 时查找相机驱动包，所以仍须安装该包。此命令会启动 ROS 节点，不能当作纯离线检查。

完成设备标定和现场净空检查后，默认命令会自动运动并只执行一次抓取：

```bash
ros2 launch rgbd_grasp rgbd_grasp.launch.py
```

动作顺序为观察姿态、等待/有界搜索、水平对准、全路径 IK/FK 检查、接近、闭爪、收拢保持。末态 `holding_unconfirmed` / `auto_grasp_complete` 表示程序动作完成，仍须确认实物夹稳。详细参数与停止接口见 [抓取说明](src/rgbd_grasp/README.md)。

## 验证

```bash
python3 tools/check_publication.py
python3 tools/run_offline_tests.py
```

检查脚本只检查源码、打包入口和固定版本摘要；离线测试使用合成数据，不连接机器人。发布时的已执行项目、未执行项目及环境限制见 [验证记录](docs/VALIDATION.md)。ROS 集成、Nav2 原生插件构建、手柄断连恢复和实物抓取需要相应环境另行验收。

## 发布范围

本次从原工作区导出自研层，没有发布其父仓库历史。详细排除项见 [逐项范围清单](docs/publication-scope.json)。厂商代码没有被本仓库许可证重新授权。

子目录中保留的项目文档记录了特定时间和设备上的操作条件，其中路径、版本摘要及实验结果不能视为其他设备的验收结果。
