#!/usr/bin/env bash
# 环境变量加载 (需 source, 不要直接执行):
#   source tool/scripts/setup_env.sh
#   source tool/scripts/setup_env.sh --no-proxy    # 同时清掉终端里的 http(s)_proxy
#
# 作用:
#   1. 加载 ROS 2 Humble
#   2. 若已 colcon build, 叠加本工作区 install/
#   3. 设置 GAZEBO_MODEL_PATH (克隆仓库 + install 内的模型)

# 本脚本位于 <ws>/tool/scripts/, 上溯两层才是工作区根
ROBOCON_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROBOCON_WS

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

if [ -f "${ROBOCON_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${ROBOCON_WS}/install/setup.bash"
else
    echo "[提示] 尚未 colcon build, 运行: cd ${ROBOCON_WS} && colcon build --symlink-install"
fi

_PATHS="${ROBOCON_WS}/tool/gazebo_models:${ROBOCON_WS}/install/robocon_gazebo/share/robocon_gazebo/models"
export GAZEBO_MODEL_PATH="${_PATHS}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
unset _PATHS

if [ "${1:-}" = "--no-proxy" ]; then
    unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
    echo "[提示] 已清理代理环境变量 (Gazebo 需要)"
fi

echo "[环境] ROS_DISTRO=${ROS_DISTRO}  GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
