# 视觉ROS2接入、识别映射与双相机并行排查

## 工作台嵌入检测画面（2026-09-20第45节）

导航页向下滚动至“识别区 · 方块类别 → 园区”，点击“开始画面预览”。它按需启动现有ROS2适配节点，串行抓拍/推理，完成一帧后至少等待1秒再刷新；无需先选择地图或到达识别区，不发底盘或机械臂指令。点击“停止画面预览”即取消显示和后续请求。导航、联动等任务优先，独立预览自动退出；正式识别阶段使用其已有请求更新相同画面，不额外拍第二份照片。

画面显示检测框、T/G/blue或未知、置信度。只有左右各一个有效T/G标识且框不交叠时才标注“园区1候选/园区2候选”；正式采用结果仍看下方T/G对应点名和“连续3次有效一致”，需原识别区2cm/10°停车验收。预览本身不会采用或发布园区映射。空检测显示原相机画面和“未检测到物品”；失败/取消清除画面，超过5秒的图片明确标注旧帧。

新增`/raicom/vision/image/compressed`（sensor_msgs/CompressedImage，reliable/volatile）；推理结果JSON的`reply.preview`包含该图的ROS时间戳及SHA256。客户端只配对当前请求成功回复和对应图像，支持图像/JSON乱序，不接收取消后的迟到结果。JPEG是同一适配锁内、推理完成后读取的输入照片，框由工作台绘制；JSON及事件日志不包含图像二进制。额外取图失败在`preview_error`报告，原分类回复仍有效。时戳是适配接收时间，不是曝光时间；原网页不受跨进程锁控制，联动时不能另外触发网页抓拍。原相机、模型、参数及Docker配置未改。

显示使用Pi已有Pillow/ImageTk；实现参照[ImageTk官方文档](https://pillow.readthedocs.io/en/stable/reference/ImageTk.html)。第45节本地/Pi回归与真实ROS/HTTP图片配对通过，但真实T/G分类与左右互换仍待现场实物验收。部署及当前运行状态见CURRENT第45节。

2026-09-20已获用户连接和实机修改授权，并安装到MentorPi生产工作区。相机及YOLO容器的代码、模型、参数与配置保持冻结。ROS2适配节点已在私有域88完成真实HTTP抓拍/推理及Aurora并行测试，测试节点结束后退出；可从工作台“启动视觉节点”按需启动。

本轮并行结果：原厂`aurora930_launch.py`运行时收到217帧深度、169帧RGB，均640×400；18轮双端口GET共36次均200，Windows侧两个网页也可达。ROS抓拍成功发布26689字节JPEG，推理成功但`detections=[]`。因此只确认这次有界并行与接口链路，不表示T/G实物分类验收通过，也未复现或根治用户此前的超时。原厂相机默认启动不等于同学完整夹取进程的负载条件。内核本轮启动期有Aurora USB重连及error -71，属于待关联线索；未修改USB、相机或容器配置。完整证据及boot见CURRENT第41节。

## 接入方式

在已有ROS2 Humble环境（历史为MentorPi）运行两个HTTP适配节点。Docker服务继续负责实际相机和模型；适配节点不加载Picamera2、不打开`/dev/video*`、不启动第二份相机驱动。深度相机仍由同学的ROS2驱动独立使用。

```text
场地工作台 ── String/JSON ── yolo_infer ROS2节点 ── POST 8080/api/do_infer
                                                   └─ 原服务内部抓拍8000 → YOLO
ROS2相机请求 ─────────────── yolo_camera ROS2节点 ─ POST 8000/api/capture_temp
                                                   └─ GET 8000/api/temp_image → JPEG话题
```

本轮重新核对Docker名为`pi_cam_env`、`yolo_env`，yolo_env共享pi_cam_env网络和IPC，8000/8080端口发布在相机容器；MentorPi使用host网络，因此适配节点可用127.0.0.1访问。换容器或网络后须重新核对，从另一个bridge容器访问127.0.0.1并不等于访问Pi宿主。[Docker网络与共享网络命名空间](https://docs.docker.com/engine/network/)

代码已安装到`/home/ubuntu/ros2_ws/install/raicom_control`。source原生产工作区后，可点击工作台“启动视觉节点”，或在同一ROS2环境运行：

```bash
ros2 launch raicom_control vision_http.launch.py
```

入口为`raicom_control.vision_ros_nodes:camera_main`和`infer_main`。launch仅启动这两个节点，不启动或改造Docker。允许显式传入`camera_url:=http://<核实后的地址>:8000`和`infer_url:=http://<核实后的地址>:8080`，不固化历史Wi-Fi地址。同一个ROS graph中各保持一份；检测到已有节点时UI不重复启动。识别等待期间也能点击启动节点。

| ROS接口 | 类型 | 行为 |
|---|---|---|
| `/raicom/camera/request`、`/result` | std_msgs/String，JSON | 带task_id/request_id的抓拍、结果和取消 |
| `/raicom/camera/capture` | std_srvs/Trigger | 立即返回是否入队及请求编号，实际结果异步发布 |
| `/raicom/camera/image/compressed` | sensor_msgs/CompressedImage | 每次成功抓拍发布JPEG；不是持续视频流 |
| `/raicom/camera/status` | std_msgs/String，JSON | GET参数接口可达性；不是抓拍验收 |
| `/raicom/vision/request`、`/result` | std_msgs/String，JSON | 带编号的推理、结果和取消 |
| `/raicom/vision/infer` | std_srvs/Trigger | 立即返回入队状态，最终推理结果在result话题 |
| `/raicom/vision/status` | std_msgs/String，JSON | GET推理主页可达性；不是分类验收 |
| `/raicom/vision/park_mapping` | std_msgs/String，JSON | 工作台采用或撤销的本轮类别→园区→点位映射 |

请求和结果采用reliable/volatile，映射采用reliable/transient_local depth=1，订阅端须设置兼容QoS。示例请求：

```json
{"task_id":"task-current","request_id":"unique-request-001","command":"start"}
```

取消沿用同一个task_id/request_id，将command改为`cancel`。结果示例：

```json
{"task_id":"task-current","request_id":"unique-request-001","status":"success","reply":{"payload":{"status":"success","image_size":[648,486],"detections":[]},"timestamp_is_request_time_not_exposure":true}}
```

结果status还可能为error或cancelled，带error信息。上例空detections只表示一次接口返回，不会通过园区识别。底层无曝光时间戳，JPEG ROS时间戳是适配节点收到图像时刻，不能用于普通相机与深度图的精确同步或共用像素坐标。

HTTP在单独工作线程执行，ROS回调不等待HTTP；客户端只接收当前task_id/request_id，超时或取消后丢弃迟到结果。取消不能强制终止已经进入原HTTP服务的抓拍/推理，只撤销结果使用。相机/推理节点在同一个ROS运行环境中共享文件锁，避免同时改写临时图。原网页和旧`yolo_roi_bridge`不受这个锁控制，联动期间不要同时触发它们；不要把同一相机的适配节点拆到不同/tmp命名空间。后台GET健康检查不抓拍。

## 识别区映射阶段

先在界面选择识别点、A=园区1目的地、B=园区2目的地。识别点必须已经采集，A/B必须是当前地图两个不同正式点。

到识别区后先停车复核2cm/20°，再连续请求三次有效且一致的结果：每帧必须有一个T和一个G，左右框不能交叠，标签/置信度/框范围须有效；空帧、未知类别、失败或不一致会重置连续计数。`blue`不属于配送类别。HTTP字符串`1→T、2→G`，左侧标识对应A，右侧对应B。

若左G、右T，界面显示：

```text
G → 园区1（A） → 当前所选A点名
T → 园区2（B） → 当前所选B点名
```

相应映射JSON（省略时间和几何指纹）：

```json
{"schema":"raicom_station_mapping/v1","valid":true,"task_id":"current-task","mapping_id":"unique-mapping","station":"shibie","category_to_park":{"G":"A","T":"B"},"category_to_point":{"G":"pick_A","T":"pick_B"},"label_to_park":{"1":"B","2":"A"},"confirmed_frames":3,"scope":"this_task_only"}
```

同学收到PICK_READY后，若其最终分类是HTTP标签1/2，可根据当前任务的`label_to_park`决定返回A或B。若同学已经输出最终A/B，本项目按园区直接消费，不能再交换一次。`/raicom/task/status`也提供task_id及mapping（标签1/2到A/B）。映射消费者必须同时核对valid、task_id及当前任务状态；撤销消息为`valid:false`，不可沿用上轮映射或把独立预览映射用于配送任务。

每轮联动都会重新识别，地图/目的地/识别点修改或取消会撤销显示与发布。独立“识别当前园区映射”仅停车识别，不导航、不夹取；60秒内未取得一致结果即失败。完整任务仍是中转→识别→left_a两次/right_a两次→A/B园区放置，支持后续扩到8件。运输/放置标定和实物确认要求不变。

## 两网页均打不开时的排查

**当前只知道：并行启动深度驱动后8000/8080网页都打不开或超时。根因尚不明，增加ROS2包装不能修复HTTP服务本身不可达。**

接车后分别记录正常状态和故障状态，保持现场已经存在的进程，本脚本不负责启动/停止深度驱动、不恢复服务：

```bash
python3 deploy/camera_coexistence_audit.py --label before_depth --output /tmp/raicom-camera-before.json
python3 deploy/camera_coexistence_audit.py --label with_depth_failure --output /tmp/raicom-camera-failure.json
```

脚本只读取容器状态/网络/端口/进程/日志、内存负载、USB拓扑、video设备持有者、内核USB/OOM线索，并从Pi宿主及相关容器分别GET两个端口。无POST抓拍、无设备采集、无容器重启、无USB复位、无底盘/舵机输出。写出的只是指定诊断JSON，已有同名文件拒绝覆盖；权限不足或命令不存在会保留错误，不自动提权修复。

| 证据 | 下一步定位 |
|---|---|
| 容器内GET也失败、无监听进程 | 看应用退出/卡住、启动前相机或模型初始化、日志、OOM或资源耗尽 |
| 容器内正常、Pi宿主失败 | 查网络共享对象、端口发布、容器重建后的命名空间关系 |
| Pi宿主正常、电脑浏览器失败 | 查当前Pi地址、Wi-Fi链路、访问URL及防火墙，不先改模型 |
| 深度启动后出现USB断连、OOM、欠压 | 按对应日志继续定位USB/电源/内存/CPU；不能只凭同时发生判根因 |
| HTTP主页正常但推理失败 | 再看抓拍返回、临时图/共享IPC和YOLO业务error；这不同于网页不可达 |

本地vendor launch还有一条需核对的分支：`DEPTH_CAMERA_TYPE`非aurora/ascamera时可能落到usb_cam。实际使用哪个驱动、哪个设备和发布哪些图像流须读现场进程/env，不能假定一定是Aurora，也不能据此直接改参数。

## 本地验证与待实机部分

离线测试覆盖HTTP方法/大小/错误、编号关联与取消、异步Trigger、取消后不发布图像、三帧一致与左右互换、四/八件流程及停车调速的写入/回读失败。调速最新扩展为控制器/平滑器/guard三节点，软件线速度上限2.0m/s，角速度可调；详见工作台候选说明。输入为Mock；没有把测试图或界面示例当真实分类。

仍待连接后验证：ROS2 Humble安装/启动和QoS；现有HTTP正常及双相机并行可用；T/G各10帧（每类≥9有效，所有有效正确）及左右互换；原生Nav2插件编译/加载；真实2cm/20°精度、速度、夹持和落点。原容器冻结边界继续有效。

实现依据：[ROS2 Humble回调组/避免阻塞](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/How-To-Guides/Using-callback-groups.rst)、[Humble DWB动态限速](https://github.com/ros-navigation/navigation2/blob/1.1.15/nav2_dwb_controller/dwb_plugins/src/kinematic_parameters.cpp)、[Humble速度平滑器动态参数](https://github.com/ros-navigation/navigation2/blob/1.1.15/nav2_velocity_smoother/src/velocity_smoother.cpp)。Context7检索配额已耗尽，本轮使用上述官方源码核对；本地代码验证不等于现场插件行为已经通过。
