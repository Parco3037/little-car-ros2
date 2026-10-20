#!/usr/bin/env bash
set -Ee -o pipefail

# One operator session: existing unique base -> 2 cm SLAM -> guarded manual
# control -> explicit save/discard. It never starts a second hardware stack.

MAP_NAME="${1:-}"
MAP_DIR="${RAICOM_MAP_DIR:-/home/ubuntu/ros2_ws/src/slam/maps}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="${RAICOM_MAPPING_RVIZ:-/home/ubuntu/ros2_ws/src/raicom_control/config/slam_low_load.rviz}"
LOG_DIR="${RAICOM_LOG_DIR:-/home/ubuntu/raicom_runtime/mapping_sessions}"

if [[ ! "$MAP_NAME" =~ ^[A-Za-z0-9_-]{1,80}$ ]]; then
  echo "用法: $0 新地图名（只允许字母、数字、下划线、短横线）" >&2
  exit 2
fi
if [[ -e "$MAP_DIR/$MAP_NAME.yaml" || -e "$MAP_DIR/$MAP_NAME.pgm" ]]; then
  echo "拒绝覆盖已有地图: $MAP_DIR/$MAP_NAME.(yaml|pgm)" >&2
  exit 3
fi
[[ -f "$RVIZ_CONFIG" ]] || { echo "RViz 配置不存在: $RVIZ_CONFIG" >&2; exit 3; }
[[ -f "$TOOLS_DIR/guard_lease_publisher.py" ]] || {
  echo "缺少安全许可工具: $TOOLS_DIR/guard_lease_publisher.py" >&2; exit 3;
}
[[ -f "$TOOLS_DIR/mapping_arm_rest.py" && -f "$TOOLS_DIR/mapping_rest_pose.json" ]] || {
  echo "缺少建图休息姿态工具或参数，拒绝启动" >&2; exit 3;
}

source /home/ubuntu/ros2_ws/install/setup.bash
set -u
mkdir -p "$LOG_DIR"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SESSION_DIR="$LOG_DIR/${MAP_NAME}_${STAMP}"
mkdir -p "$SESSION_DIR"

topic_info() { timeout 8s ros2 topic info "$1" 2>/dev/null || true; }
require_one_publisher() {
  local info
  info="$(topic_info "$1")"
  if ! grep -Fq 'Publisher count: 1' <<<"$info"; then
    echo "要求 $1 恰好一个发布者，当前状态:" >&2
    printf '%s\n' "$info" >&2
    exit 4
  fi
}
require_zero_once() {
  local sample values
  sample="$(timeout 8s ros2 topic echo /controller/cmd_vel --once 2>/dev/null || true)"
  values="$(awk '/^[[:space:]]+(x|y|z):/{print $2}' <<<"$sample")"
  if [[ "$(wc -w <<<"$values")" -ne 6 ]] ||
      ! awk '{for(i=1;i<=NF;i++) if (($i+0) != 0) exit 1}' <<<"$values"; then
    echo "无法证明 /controller/cmd_vel 当前为全零，拒绝启动" >&2
    printf '%s\n' "$sample" >&2
    exit 4
  fi
}
wait_tf() {
  local output=''
  for _ in {1..8}; do
    output="$(timeout 3s ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 || true)"
    grep -Fq 'Translation:' <<<"$output" && return 0
  done
  echo "缺少 TF $1 -> $2" >&2
  printf '%s\n' "$output" >&2
  return 1
}
wait_one_publisher() {
  local info=''
  for _ in {1..30}; do
    info="$(topic_info "$1")"
    grep -Fq 'Publisher count: 1' <<<"$info" && return 0
    sleep 1
  done
  echo "$1 未建立唯一发布者" >&2
  printf '%s\n' "$info" >&2
  return 1
}
wait_motor_subscriber() {
  local info=''
  for _ in {1..30}; do
    info="$(topic_info /ros_robot_controller/set_motor)"
    grep -Fq 'Subscription count: 1' <<<"$info" && return 0
    sleep 1
  done
  echo "底盘控制器没有订阅 /ros_robot_controller/set_motor；真车不会运动" >&2
  printf '%s\n' "$info" >&2
  return 1
}
stop_group() {
  local pid="${1:-0}"
  (( pid > 0 )) || return 0
  kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
  # Let ros2 launch receive SIGINT first and reap its own children. Signalling
  # the whole group at once leaves defunct children under this container PID1.
  kill -INT "$pid" 2>/dev/null || true
  for _ in {1..80}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -TERM -- "-$pid" 2>/dev/null || true
  for _ in {1..30}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

nodes="$(timeout 8s ros2 node list 2>/dev/null || true)"
for forbidden in /slam_toolbox /keyboard_input /manual_control_panel /map_server /amcl /controller_server; do
  if grep -qx "$forbidden" <<<"$nodes"; then
    echo "拒绝重复启动已有节点: $forbidden" >&2
    exit 4
  fi
done
for required in /ros_robot_controller /cmd_vel_guard /odom_publisher; do
  grep -qx "$required" <<<"$nodes" || {
    echo "缺少基础节点: $required" >&2
    exit 4
  }
done
wait_motor_subscriber
require_one_publisher /scan
require_one_publisher /odom
require_one_publisher /controller/cmd_vel
require_zero_once
wait_tf odom base_footprint
python3 -B - <<'PY'
import tkinter as tk
root = tk.Tk()
root.withdraw()
root.update_idletasks()
root.destroy()
print('图形界面连接正常')
PY

slam_pid=0
rviz_pid=0
lease_pid=0
panel_pid=0
cleanup() {
  set +e
  stop_group "$lease_pid"
  stop_group "$panel_pid"
  stop_group "$rviz_pid"
  stop_group "$slam_pid"
}
trap cleanup EXIT
trap 'cleanup; exit 143' HUP TERM

read -r -p "机械臂将缓慢收至建图休息姿态；确认车体停稳、手已离开摆动范围，输入 yes: " arm_clear
[[ "$arm_clear" == yes ]] || { echo "未确认机械臂净空，未启动建图"; exit 4; }
echo "执行建图休息姿态并读取真实舵机位置；失败不会开放遥控。"
python3 -B "$TOOLS_DIR/mapping_arm_rest.py" --apply | tee "$SESSION_DIR/arm_rest.log"
require_zero_once

echo "启动 2 cm SLAM，日志: $SESSION_DIR/slam.log"
setsid ros2 launch raicom_control home_slam.launch.py >"$SESSION_DIR/slam.log" 2>&1 &
slam_pid=$!
wait_one_publisher /map
wait_tf map base_footprint

echo "启动 RViz；白色激光使用唯一正式 /scan，灰白模型跟随 map -> base_footprint。"
setsid env LP_NUM_THREADS=1 dbus-run-session -- nice -n 10 rviz2 -d "$RVIZ_CONFIG" >"$SESSION_DIR/rviz.log" 2>&1 &
rviz_pid=$!
sleep 4
kill -0 "$rviz_pid" 2>/dev/null || { echo "RViz 启动失败" >&2; exit 5; }

echo "启动临时运动许可；仅本次建图会话有效。"
setsid python3 -B "$TOOLS_DIR/guard_lease_publisher.py" \
  --mission-id "mapping-${MAP_NAME}-${STAMP}" >"$SESSION_DIR/lease.log" 2>&1 &
lease_pid=$!

echo "启动可点击面板和键盘：W/S 前后，A/D 横移，Q/E 旋转，Space 停车。"
setsid ros2 run raicom_control manual_control_panel >"$SESSION_DIR/panel.log" 2>&1 &
panel_pid=$!
sleep 2
kill -0 "$panel_pid" 2>/dev/null || { echo "控制面板启动失败" >&2; exit 5; }

set +e
ros2 run raicom_control keyboard_input
keyboard_status=$?
set -e
stop_group "$lease_pid"; lease_pid=0
stop_group "$panel_pid"; panel_pid=0
sleep 1
require_zero_once
if (( keyboard_status != 0 && keyboard_status != 130 )); then
  echo "键盘控制异常退出，未保存地图，状态=$keyboard_status" >&2
  exit 6
fi

while true; do
  read -r -p "输入 save 保存地图；输入 discard 放弃本次地图: " decision
  case "$decision" in
    save)
      ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" \
        --ros-args -p map_subscribe_transient_local:=true \
        | tee "$SESSION_DIR/map_saver.log"
      [[ -s "$MAP_DIR/$MAP_NAME.yaml" && -s "$MAP_DIR/$MAP_NAME.pgm" ]] || {
        echo "地图文件未完整生成" >&2; exit 7;
      }
      grep -Eq '^resolution:[[:space:]]*0\.02([0-9]*)?[[:space:]]*$' \
        "$MAP_DIR/$MAP_NAME.yaml" || {
          echo "保存后的地图不是 2 cm 分辨率，拒绝标记完成" >&2; exit 7;
        }
      sha256sum "$MAP_DIR/$MAP_NAME.yaml" "$MAP_DIR/$MAP_NAME.pgm" \
        | tee "$SESSION_DIR/map_sha256.txt"
      echo "地图保存完成: $MAP_DIR/$MAP_NAME.yaml"
      break
      ;;
    discard)
      echo "已放弃本次地图，没有生成或覆盖地图文件。"
      break
      ;;
    *) echo "只能输入 save 或 discard" ;;
  esac
done
echo "建图会话结束；证据: $SESSION_DIR"
