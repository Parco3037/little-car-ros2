#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="MentorPi"
TOOLS_DIR="/home/ubuntu/raicom_tools"
MAP_NAME="${1:-}"
if [[ ! "$MAP_NAME" =~ ^[A-Za-z0-9_-]{1,80}$ ]]; then
  echo "用法: ~/raicom_mapping.sh 新地图名" >&2
  exit 2
fi
[[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" == true ]] || {
  echo "MentorPi 未运行" >&2; exit 3;
}
require_chassis() {
  if [[ ! -e /dev/rrc || ! -e "$(readlink -f /dev/rrc 2>/dev/null || true)" ]] ||
      ! lsusb | grep -qi '1a86:55d4'; then
    echo "底盘串口未在线：需要 /dev/rrc -> /dev/ttyACM0 和 USB 1a86:55d4。" >&2
    echo "请先检查底盘 USB 线、控制器供电和接口，看到 /dev/ttyACM0 后再启动建图。" >&2
    exit 3
  fi
}
require_chassis
holders="$(sudo -n fuser /dev/rrc 2>/dev/null || true)"
read -r -a holder_ids <<<"$holders"
if [[ ${#holder_ids[@]} -ne 1 ]] ||
    ! tr '\0' ' ' <"/proc/${holder_ids[0]:-0}/cmdline" | grep -q '/ros_robot_controller/ros_robot_controller '; then
  echo "底盘串口不是 ROS 独占。请关闭厂家 arm 机械臂调试窗口，再点 SLAM。" >&2
  exit 3
fi
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/pi/.Xauthority}"
[[ -S /tmp/.X11-unix/X0 && -f "$XAUTHORITY" ]] || {
  echo "Pi 桌面尚未就绪，无法打开 RViz/控制面板" >&2; exit 3;
}
grant_x() { xhost +SI:localuser:pi >/dev/null; }
revoke_x() { xhost -SI:localuser:pi >/dev/null 2>&1 || true; }
trap revoke_x EXIT
trap 'revoke_x; exit 143' HUP TERM
grant_x
docker exec -it -u ubuntu -w /home/ubuntu \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 "$CONTAINER" \
  python3 "$TOOLS_DIR/subreaper_supervisor.py" \
  /bin/bash "$TOOLS_DIR/raicom_mapping_session.sh" "$MAP_NAME"
