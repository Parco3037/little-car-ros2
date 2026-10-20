# 现有摄像头 / YOLO 容器接口实查

**2026-09-20补充：用户已确认连接并授权修改，ROS2 HTTP适配与三帧园区映射已安装。本轮原厂Aurora驱动与两个HTTP服务有界并行通过，ROS抓拍和推理成功，但detections仍为空；未改/重启原容器，也未复现此前网页超时。以下正文保留9月15日历史实查，今天的boot、帧数、响应及限制见 [VISION_ROS2_INTEGRATION.md](VISION_ROS2_INTEGRATION.md) 和CURRENT第41节。pick内部深度识别/夹取由同学完整流程负责。**

检查日期：Windows 2026-09-15；Pi 回执时间 2026-09-14 23:28 +08:00，双方时钟不一致。
本次实际 SSH：`pi@<ROBOT_OR_HOST_IP>`，boot `ecdad943-b012-4b1e-9be2-8a384015b8dc`。
以下来自当前容器挂载的代码及一次真实调用，不以旧 reference 副本或网页名称代替接口证据。

## 三个运行容器的职责

| 容器 | 本次运行进程 / 入口 | 职责 |
| --- | --- | --- |
| MentorPi | 原 base_production、cmd_vel_guard 等 ROS2 进程 | 底盘、雷达、定位/导航控制层；本次未发现 Aurora 驱动或 Nav2 进程，不代表已完成 ROS 数据流验收 |
| pi_cam_env | `/app/camera_app.py`，宿主 `/home/pi/camera_project/camera_app.py` | Picamera2 普通相机抓拍，HTTP 8000 |
| yolo_env | `/app/infer_web.py`，宿主 `/home/pi/yolo_project/infer_web.py` | 对上述照片做 ONNX 检测，HTTP 8080 |

`yolo_env` 共享 `pi_cam_env` 的网络与 IPC，因此 Docker 端口列表中 8000/8080 都显示在 `pi_cam_env` 下；这不表示 YOLO 进程在相机容器内。二者通过 `/dev/shm/temp.jpg` 交接照片。

## 实际接口

外部地址前缀为 `http://<ROBOT_OR_HOST_IP>`；MentorPi 使用 host 网络，可通过 `http://127.0.0.1` 调用。以后换网需重新发现主机，不能继续硬编码旧 `<ROBOT_OR_HOST_IP>`。

| 方法 | 端口及路径 | 输入和结果 |
| --- | --- | --- |
| GET | `8000/` | 相机控制页面 |
| GET | `8000/api/params` | 当前相机参数；本次 648×486，曝光 30000，模拟增益 2.5 |
| POST | `8000/api/capture_temp` | 无需请求体，拍一张临时照片；成功 HTTP 200 / `{"status":"success"}`，失败 HTTP 500 |
| GET | `8000/api/temp_image` | 最近临时 JPEG；本接口自身不拍新照片 |
| POST | `8080/api/do_infer` | 无需请求体，自动调用上面的抓拍接口，读取共享照片并推理 |
| GET | `8080/latest_result.jpg` | 最近画框结果图；本接口自身不重新推理 |
| GET | `8080/` | YOLO 页面 |

相机还有 `GET /api/gallery`、`GET /photos/<name>`、`POST /api/save_temp`、`POST /api/discard_temp`、`POST /api/params`。后三个会保存/删除照片或修改配置，本次未调用。代码没有提供上传 Aurora 图像进行推理、深度查询或指定目标框的 HTTP 接口。

`POST /api/do_infer` 的正常返回结构：

```json
{
  "status": "success",
  "detections": [],
  "image_size": [648, 486],
  "timing": {"capture": 558, "read": 36, "preprocess": 31, "inference": 319}
}
```

这是本次实际返回，空数组表示本帧无检测结果，不能宣布 T/G 识别通过。总 HTTP 耗时约 0.954 秒；四个 timing 值单位为毫秒。结果图偏暗，未见清楚的待识别方块。

有检测结果时，每条结构为 `{"class": <字符串>, "score": <数值>, "bbox": [x1,y1,x2,y2]}`，框坐标为原始照片像素。当前阈值为 `score > 0.4`。该接口业务失败也可能返回 HTTP 200，必须检查 `status` 和 `error`，不能仅检查 HTTP 状态。

## 类别：模型索引与 API 标签不能混用

当前 `infer_web.py` 第 12 行为 `CLASSES = ["1", "2", "blue"]`，第 243–252 行将模型索引转换成字符串；接口没有直接返回 `class_id` 或 T/G。

| 模型输出索引 | HTTP 返回 class | 按用户此前确认的任务含义 |
| --- | --- | --- |
| 0 | `"1"` | T |
| 1 | `"2"` | G |
| 2 | `"blue"` | 蓝色方块类别 |

右列沿用用户给出的 0=T、1=G，不是本次空检测图证明的模型语义。ROS 适配层应明确按 API 标签 `"1"→T`、`"2"→G` 转换，保留原标签供诊断。不能把 HTTP 的 `"1"` 当作模型索引 1，从而错映射为 G；`blue` 不能当作 T/G。

## 对 pick 流程的实际影响

用户指定顺序：导航到 pick 停车 → Aurora RGB/深度找蓝色方块及相对位置 → 锁定同一方块并对齐底盘 → 停稳 → 调用现有相机/YOLO 接口识别 T/G → 按位置或高度选择抓取姿态 → 夹取。

这里的视觉纠偏是车体与目标的相对对齐，不把可移动蓝色方块用作 AMCL 绝对地图初值，也不改地图或正式点位。

现有 API 是普通相机二维检测，不能直接返回 Aurora 深度。Aurora RGB/depth 历史输出 frame 不同，尚无本轮配准证据；普通相机与 Aurora 更不能直接复用像素框。接入仍需核验相机外参/对应关系、同一目标锁定、对齐距离和机械臂标定。

当前 API 不返回采集时间、request_id 或目标跟踪 ID，也不接受外部图片；因此控制层需要串行调用、记录请求边界，并在静止及目标一致性检查后使用分类结果。现有 `yolo_roi_bridge.py` 默认地址与本次接口相符，但仅做唯一目标/ROI 桥接，没有实现蓝色优先的底盘对齐和八件配送闭环。

## 本轮操作与证据

仅检查运行状态/代码/参数，额外调用一次已有推理接口以验证返回；该调用按原应用行为更新了临时照片和 `latest_result.jpg`。未改代码、ONNX 模型、配置、容器镜像/网络/启动设置，未重启容器，未发送导航、底盘或机械臂指令。

检查前后代码、模型、参数 SHA 相同，三个容器 ID、镜像 ID、启动时间与 restart count 相同。相关文件：

- [运行容器与进程](../runtime_evidence/20260915_vision_api_audit/container_inventory.json)
- [本次 API 实际返回及文件 SHA](../runtime_evidence/20260915_vision_api_audit/api_live_probe.json)
- [不变性核验](../runtime_evidence/20260915_vision_api_audit/unchanged_check.json)
- [本次结果图](../runtime_evidence/20260915_vision_api_audit/inference_result.jpg)
- [相机代码只读副本](../runtime_evidence/20260915_vision_api_audit/camera_app.py)
- [YOLO 代码只读副本](../runtime_evidence/20260915_vision_api_audit/infer_web.py)

相机代码 SHA `dbac2ab5f819e43601131a25378fedeff60aed0ada97d217afbc9952061f85ee`；YOLO 代码 SHA `7a29f3196c5f300d601d103c647dca47d1d882a1c62a3ae0acbf9e8eb6de346e`。
