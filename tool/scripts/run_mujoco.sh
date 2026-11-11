#!/usr/bin/env bash
# MuJoCo Dog 仿真启动包装
#   - 使用专用 venv ($HOME/.venvs/robocon_mujoco) 的 python3.10
#   - 透传参数: --duration N / --no-policy / --xml / --onnx
# 用法: ./tool/scripts/run_mujoco.sh
#       ./tool/scripts/run_mujoco.sh --duration 10
#       ./tool/scripts/run_mujoco.sh --no-policy
set -e

# 本脚本位于 <ws>/tool/scripts/, 上溯两层才是工作区根
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${ROBOCON_VENV:-$HOME/.venvs/robocon_mujoco}"
SCRIPT="${WS}/tool/gazebo_models/robocon_mujoco/play_robocon.py"

[ -x "${VENV}/bin/python" ] || {
    echo "[错误] 找不到 venv: ${VENV}"
    echo "  重建: /usr/bin/python3 -m venv ${VENV} && ${VENV}/bin/pip install mujoco onnxruntime pyyaml numpy evdev"
    exit 1
}

if ! "${VENV}/bin/python" -c "import mujoco, onnxruntime" 2>/dev/null; then
    echo "[错误] venv 内缺少 mujoco / onnxruntime, 请执行:"
    echo "  ${VENV}/bin/pip install -i https://pypi.tuna.tsinghua.edu.cn/simple mujoco onnxruntime pyyaml numpy evdev"
    exit 1
fi

exec "${VENV}/bin/python" "${SCRIPT}" "$@"
