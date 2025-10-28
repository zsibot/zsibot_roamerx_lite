#!/bin/bash
# ============================================================
# 终止 ROS2 Navigation Stack 与 RViz2 相关进程
# ============================================================

echo ">>> 正在查找并终止导航相关进程..."

PROCESS_NAMES=(
  "ros2 launch robot_navigo navigation_bringup.launch.py"
  "rviz2"
  "navigation_bringup"
)

for pname in "${PROCESS_NAMES[@]}"; do
  pids=$(pgrep -f "$pname")
  if [ -n "$pids" ]; then
    echo ">>> 终止进程: $pname (PID: $pids)"
    kill -9 $pids 2>/dev/null
  else
    echo ">>> 未检测到进程: $pname"
  fi
done

leftover=$(pgrep -f "ros2")
if [ -n "$leftover" ]; then
  echo "  检测到仍有 ROS2 相关进程存在: $leftover"
  echo "  如需强制清理，可执行:  kill -9 $leftover"
else
  echo "  所有导航相关进程已终止。"
fi
