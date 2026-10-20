#!/usr/bin/env bash
set -Eeuo pipefail

# Desktop entry for the RAICOM mapping workflow.
# The actual ROS session is kept in raicom_mapping.sh; this file only provides
# an operator-friendly terminal prompt and never starts a second hardware stack.

exec 2>&1
echo "RAICOM 安全建图入口"
echo "只使用现有 MentorPi 基础链；确认净空后先执行建图休息姿态，不启动导航或抓取。"
echo "底盘 USB、零速和唯一 ROS 发布者检查失败时会自动拒绝启动。"
echo

MAP_DIR="/home/ubuntu/ros2_ws/src/slam/maps"
DEFAULT_MAP="map_lab_$(date +%Y%m%d_%H%M%S)"
read -r -p "输入新地图名（直接回车使用 ${DEFAULT_MAP}，不会覆盖旧地图）: " MAP_NAME
MAP_NAME="${MAP_NAME:-$DEFAULT_MAP}"

if [[ ! "$MAP_NAME" =~ ^[A-Za-z0-9_-]{1,80}$ ]]; then
  echo "地图名无效：只允许字母、数字、下划线和短横线。"
  read -r -p "按回车关闭窗口..." _
  exit 2
fi

if [[ ! -x /home/pi/raicom_mapping.sh ]]; then
  echo "缺少 /home/pi/raicom_mapping.sh，未启动任何 ROS。"
  read -r -p "按回车关闭窗口..." _
  exit 3
fi

# Avoid two desktop clicks creating competing SLAM/keyboard sessions.
exec 9>"/tmp/raicom_slam_desktop.lock"
if ! flock -n 9; then
  echo "已有建图会话在运行；拒绝重复启动。"
  read -r -p "按回车关闭窗口..." _
  exit 4
fi

if /home/pi/raicom_mapping.sh "$MAP_NAME"; then
  status=0
else
  status=$?
fi
if (( status == 0 )); then
  echo
  echo "建图入口已正常结束；地图是否保存由会话中的 save/discard 选择决定。"
else
  echo
  echo "建图入口未完成，退出码：$status；请看上方具体失败原因。"
fi
read -r -p "按回车关闭窗口..." _
exit "$status"
