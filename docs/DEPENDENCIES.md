# 外部依赖与资源准备

## ROS 与厂商工作空间

先安装 ROS 2 Humble 并加载厂商工作空间，确认下列实际包可被 `ros2 pkg prefix` 找到。本仓库没有复制其源码。

| 功能 | 外部依赖 |
|---|---|
| 底盘与控制 | `controller`、`peripherals`、`ros_robot_controller`、`ros_robot_controller_msgs`、`servo_controller`、`servo_controller_msgs` |
| 抓取 | `kinematics`、`kinematics_msgs`、`deptrum-ros-driver-aurora930` |
| 机器人 TF | `landerpi_description`、`joint_state_publisher`、`robot_state_publisher` |
| 感知 | `cv_bridge`、`message_filters`、NumPy、OpenCV、标准 ROS 消息及 TF |
| 手柄/语音 | `joy`，语音模式另需 `xf_mic_asr_offline`、Vosk、PyAudio、ALSA |
| 导航/建图 | Nav2、DWB、SLAM Toolbox、robot_localization，及所选配置引用的插件 |
| 桌面工具 | PyYAML、Pillow/ImageTk、Tk |

以各包 `package.xml` 和实际启用的入口为准。厂商包和相机驱动不一定有公共 rosdep 规则，不能用一次 rosdep 无报错替代它们已安装的核验。

## 相机

默认使用 `/aurora/rgb/image_raw`、`/aurora/depth/image_raw`、`/aurora/rgb/camera_info`。必须核对 RGB/深度配准、深度单位、时间戳和 RGB 内参。

当前 `rgbd_grasp.launch.py` 在求值启动开关之前解析各外部包路径，因此即使 `start_camera:=false`、`start_hardware:=false`，对应包仍需可发现。`start_camera:=true` 不会自动检测已有驱动；避免重复启动。

## 本地机器人模型

从合法取得的厂商工作空间导出展开后的 URDF，并使用本仓库生成器转换。下面的路径需要替换为本机实际路径：

```bash
python3 src/raicom_control/raicom_control/lightweight_urdf.py \
  /path/to/expanded-vendor.urdf \
  src/raicom_control/models/landerpi_lightweight.urdf \
  --package-root /path/to/landerpi_description \
  --manifest /path/to/local-evidence/lightweight.manifest.json
```

在构建前生成；如果已构建，则重新构建以安装生成文件。模型和本地证据不提交到仓库。缺少此文件时，不应启动 `lightweight_robot_description.launch.py`。

## 语音、地图与兼容库

语音模型由使用者自行取得，并核对来源和许可。`competition_manual/launch/manual.launch.py` 当前读取安装目录中的 `model` 和 `sounds`，并访问讯飞包；启用此完整入口前，把合法模型放在 `src/competition_manual/model`，把自有或获授权音频放在 `sounds` 中后重新构建。原有语音流程会在超时后请求自动模式，不能把缺少模型的启动当作完整手柄流程验收。

本仓库不发布实地地图或其路线。导航前需本地建图、设置地图路径和重新记录路线，检查定位与坐标系，不能沿用文档中历史现场坐标。

导航 staging 配置引用的 bondcpp / SLAM Toolbox 兼容层具有历史版本、架构与摘要检查。先阅读各兼容层说明，在对应系统本机构建和重新核验；不要删除摘要门控或全局替换系统库以绕过缺文件错误。
