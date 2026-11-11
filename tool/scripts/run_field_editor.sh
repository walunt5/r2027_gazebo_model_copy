#!/usr/bin/env bash
# 启动场地可视化编辑器。
#   浏览器打开 http://127.0.0.1:8790 后即可可视化修改场地,
#   点"应用到工程"会写回 model.sdf 并自动 colcon build。
set -e
# 本脚本位于 <ws>/tool/scripts/, 上溯两层才是工作区根
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="${1:-8790}"
echo "场地编辑器: http://127.0.0.1:${PORT}"
echo "工作区: ${WS}"
# -B: 不写 __pycache__，避免改了脚本却跑旧字节码
exec /usr/bin/python3 -B "${WS}/tool/scripts/field_editor_server.py" --port "${PORT}"
