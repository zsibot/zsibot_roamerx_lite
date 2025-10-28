#! /bin/bash
# ============================================================
# 启动 ROS2 Navigation Stack 与 RViz2 可视化
# 使用相对路径加载地图与 RViz 配置
# ============================================================

# 当前脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 项目根目录（上上级：script/bash -> script -> 项目根目录）
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ROS2 工作空间路径
WORKSPACE_DIR="${PROJECT_DIR}"

# 地图与RViz配置文件路径
MAP_PATH="${PROJECT_DIR}/map/map.yaml"
RVIZ_CONFIG="${PROJECT_DIR}/src/navigation/src/robot_navigo/rviz/rviz2_config.rviz"

# 检查 setup.bash 是否存在
if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  echo "【ERROR】找不到 ${WORKSPACE_DIR}/install/setup.bash"
  echo "请先执行: colcon build"
  exit 1
fi

# ============================================================
# 启动导航栈
# ============================================================
gnome-terminal -- bash -c "
  echo '>>> Launching Navigation Stack...';
  source ${WORKSPACE_DIR}/install/setup.bash
  ros2 launch robot_navigo navigation_bringup.launch.py \
      platform:=UE \
      mc_controller_type:=RL_TRACK_VELOCITY \
      communication_type:=UDP \
      map:=${MAP_PATH};
  exec bash
"

# ============================================================
# 启动 RViz2
# ============================================================
gnome-terminal -- bash -c "
    echo '>>> Launching RViz2...';
    source ${WORKSPACE_DIR}/install/setup.bash
    rviz2 -d ${RVIZ_CONFIG};
  exec bash
"