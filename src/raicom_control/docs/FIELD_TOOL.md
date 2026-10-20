# RAICOM 场地建图与导航工具

本工具复用现有 SLAM Toolbox、map_server、AMCL、Nav2、`keyboard_input`、
`nav_cmd_bridge` 和唯一 `cmd_vel_guard`，不重新实现底层导航算法。

## 入口

- Pi 桌面：`field_tool.desktop`
- 宿主脚本：`/home/pi/raicom_field_tool.sh`
- 容器内命令：`ros2 run raicom_control field_tool`
- 数据目录：`/home/ubuntu/raicom_tools/field_tool_data/{maps,routes}`

## 模式

- 建图：启动 `home_slam.launch.py`，手动 W/S、A/D、Q/E，记录草稿点，
  `map_saver_cli` 保存到时间戳目录，禁止覆盖旧地图。
- 采点/纠偏：启动固定地图定位；地图画布点击拖动设置初始位姿；
  reference/approach 双坐标可分别记录，纠正保留 history。
- 导航：路线编辑器支持拖拽/上移/下移/删除/编辑；按顺序调用
  `NavigateToPose`，每段成功后才进入下一段，不重启 AMCL/Nav2/bridge。

## 安全边界

- 自动导航强制 `linear.y=0`：`nav2_controller_straight.yaml` 和
  `nav2_params_home_straight_low_load.yaml` 只允许原地旋转加直线前进/后退。
- 底盘速度仍只从 `nav_cmd_bridge -> /raicom/teleop_cmd -> guard` 输出。
- 导航每个 goal 都有 short motion-enable lease、manual suppression 和
  goal-context；停止/失败/取消立即发布零命令。
- 工具不发布 Twist，不启动机械臂或夹爪，不覆盖已有地图/route。

## 路由文件

- schema：`raicom_field_route/v2`
- 一个地图名对应一个 `.route.json`
- `map.saved` 绑定 YAML/PGM SHA-256、resolution 和 origin
- 每个点保存 reference、approach、position_tolerance_m、
  `yaw_tolerance_deg`
- 每次纠偏追加 history

## 当前验证

- `test_field_tool_model.py`：5/5 通过
- `test_field_tool_navigation_logic.py`：3/3 通过
- Pi 容器已构建，`field_tool` executable 和 straight Nav2 launch/config
  已安装
- 尚未进行真实建图保存、AMCL 初始位姿或 Nav2 实车单点验收；任何实车动作
  需要新的现场净空、可急停和明确授权
