#!/usr/bin/env bash
# Gazebo Classic 启动包装 (适配本机环境)
#   - 清理 http(s)_proxy: Gazebo 启动时会拉取模型库, 走本地代理会卡住/报错
#   - 把模型目录加入 GAZEBO_MODEL_PATH
# 用法: ./tool/scripts/run_gazebo.sh            完整场地 + Gazebo + 塔顶摆放 GUI
#       ./tool/scripts/run_gazebo.sh --full     同上
#       ./tool/scripts/run_gazebo.sh --headless 仅 gzserver, 不生成塔顶
#       ./tool/scripts/run_gazebo.sh --legacy   兼容版薄片碰撞场地 (不含 ROS GUI)
set -e

# 本脚本位于 <ws>/tool/scripts/, 上溯两层才是工作区根
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO="${WS}/tool/gazebo_models"

[ -x "${REPO}/run_gazebo.sh" ] || { echo "[错误] 未找到 ${REPO}/run_gazebo.sh"; exit 1; }

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export GAZEBO_MODEL_DATABASE_URI=""
_PATHS="${REPO}:${WS}/src/robocon_gazebo/models:${WS}/install/robocon_gazebo/share/robocon_gazebo/models"
export GAZEBO_MODEL_PATH="${_PATHS}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
unset _PATHS

FULL_WORLD="${WS}/install/robocon_gazebo/share/robocon_gazebo/worlds/robocon_track_full.world"
[ -f "$FULL_WORLD" ] || FULL_WORLD="${WS}/src/robocon_gazebo/worlds/robocon_track_full.world"

case "${1:-}" in
  --full|"")
    shift 2>/dev/null || true
    if [ -f /opt/ros/humble/setup.bash ]; then
      # shellcheck disable=SC1091
      source /opt/ros/humble/setup.bash
    fi
    if [ -f "${WS}/install/local_setup.bash" ]; then
      # shellcheck disable=SC1091
      source "${WS}/install/local_setup.bash"
    else
      echo "[错误] 尚未构建 ROS 包，无法启动塔顶 GUI。请先运行 colcon build。"
      exit 1
    fi
    if [ "${1:-}" = "--headless" ]; then
      echo "[INFO] 无界面启动完整碰撞场地 (不生成动态塔顶)"
      exec ros2 launch robocon_gazebo robocon_gazebo.launch.py \
        gui:=false placement_gui:=false
    fi
    echo "[INFO] 启动完整场地、Gazebo 与塔顶随机摆放 GUI"
    exec ros2 launch robocon_gazebo robocon_gazebo.launch.py \
      gui:=true placement_gui:=true
    ;;
  --headless)
    shift
    exec "$0" --full --headless
    ;;
  --legacy)
    shift
    echo "[WARN] legacy 场地仍包含固定塔顶视觉，不启动塔顶 GUI"
    exec "${REPO}/run_gazebo.sh"
    ;;
  *)
    echo "[错误] 未知参数: $1 (支持: --full / --headless / --legacy)"; exit 1
    ;;
esac
