# tool/ —— 工具脚本与场地模型

这里放两样东西：**你可以直接跑的脚本**，以及**上游提供的原始场地模型**。
日常使用只需要记住三条命令，其余按需查阅。

---

## 快速上手（90% 的情况只需这些）

```bash
# ① 启动完整场地（Gazebo GUI + 天空块和地基块摆放窗口）
./tool/scripts/run_gazebo.sh

# ② 可视化修改场地（浏览器里拖拽调整，点"应用到工程"自动写回并重编译）
./tool/scripts/run_field_editor.sh

# ③ 加载环境变量（需要额外模型路径或清理代理时使用）
source tool/scripts/setup_env.sh --no-proxy
```

> 所有命令都从**当前仓库根目录**执行。

---

## 目录内容

```
tool/
├── README.md                    ← 本文件
├── scripts/                     ← 自己写的工具，可以改
│   ├── run_gazebo.sh            # Gazebo 启动包装（最常用）
│   ├── run_field_editor.sh      # 场地可视化编辑器启动
│   ├── setup_env.sh             # 环境变量（需 source，不要直接执行）
│   ├── gen_full_collision.py    # 从 DAE 生成完整碰撞 model.sdf
│   ├── strip_fixed_tower_tops.py# gen 的前置步骤：剥掉固定塔顶节点
│   ├── field_editor_server.py   # 编辑器后端（标准库，无第三方依赖）
│   ├── field_editor/index.html  # 编辑器前端（单文件）
│   ├── run_mujoco.sh            # MuJoCo 机器狗演示
│   └── selftest_mujoco.py       # MuJoCo 无界面自检
└── gazebo_models/               ← 上游仓库，只读，不要改
    ├── robocon_ground/          # Gazebo 场地模型（DAE 视觉 + 薄片碰撞原始版）
    ├── robocon_mujoco/          # MuJoCo 场地 + 机器狗模型 + ONNX 策略
    ├── robocon_track.world      # 兼容版 world（薄片碰撞）
    └── run_gazebo.sh            # 上游自带的启动脚本
```

---

## 脚本逐个说明

### `run_gazebo.sh` — 启动场地

```bash
./tool/scripts/run_gazebo.sh              # 完整场地 + Gazebo GUI + 天空块和地基块摆放 GUI
./tool/scripts/run_gazebo.sh --headless   # 只开 gzserver，不生成塔顶（服务器/CI 用）
./tool/scripts/run_gazebo.sh --legacy     # 兼容版薄片碰撞场地（上游原版，仅对比用）
```

脚本会做三件事：清掉 `http(s)_proxy`（否则 Gazebo 拉模型库会卡死）、把 `tool/gazebo_models`
和 `install/` 里的模型加进 `GAZEBO_MODEL_PATH`、然后调 launch 启动。

**前置条件**：必须先 `colcon build` 过，否则会报"尚未构建 ROS 包"。

### `run_field_editor.sh` — 可视化改场地

```bash
./tool/scripts/run_field_editor.sh          # 默认 http://127.0.0.1:8790
./tool/scripts/run_field_editor.sh 9000     # 换端口
```

打开浏览器后是一张俯视图，可以：

| 操作 | 说明 |
|---|---|
| 拖拽方块 | 移动碰撞体/视觉体的位置 |
| 拖四角 | 调整尺寸 |
| 右侧数值面板 | 精确输入位置和尺寸，0.05 m 吸附（按 Shift 关闭） |
| 方向键 | 微调选中项 |
| Ctrl+Z / Ctrl+Shift+Z | 撤销 / 重做 |
| "预览" | 只看 diff，不动文件 |
| "应用到工程" | 备份 → 写回 `model.sdf` → `gz sdf -k` → 自动 `colcon build` |
| `F` / "台面着色" | 进入拉框模式，在台面上拖出矩形生成一层着色层 |

颜色是**只读**的，避免误改把场地刷花 —— 例外是"台面着色层"（`colorfilm_*`）：

- 拉框 → 自动贴到矩形中心正下方最高实体顶面（L1≈0.60、L2≈0.90、地面=0）+ 1.5 mm；
- 右栏取色器实时预览最终颜色，"贴合下方台面"重新对齐高度；
- 写回的是独立 `<visual>`：无 `<collision>`、`cast_shadows=false`、颜色只写 `<emissive>`
  （ambient/diffuse/specular 全 0，避免 Ogre 把颜色叠加 1.5~2 倍导致 Gazebo 里偏亮偏色），
  所以 Gazebo 里看到的 RGB 就是设定值，任何视角都一样，且只覆盖它正下方的顶面，不会往下包。
- `gen_full_collision.py` 重跑时会原样搬运所有 `colorfilm_*` 节点，颜色不会丢。

所有 apply 前的旧文件都备份在 `.cache/sdf_backup/`。

### `setup_env.sh` — 环境变量

```bash
source tool/scripts/setup_env.sh              # 加载 ROS 2 + install/ + 模型路径
source tool/scripts/setup_env.sh --no-proxy   # 同时清掉终端里的 http(s)_proxy
```

使用本脚本时必须 `source`，不能直接执行。常规 ROS 命令只需加载 `/opt/ros/humble/setup.bash`（若终端尚未自动加载）和 `install/setup.bash`；本脚本用于额外设置模型路径或清理代理。

### `gen_full_collision.py` — 重新生成场地碰撞

```bash
/usr/bin/python3 tool/scripts/gen_full_collision.py
```

它读 `tool/gazebo_models/robocon_ground/meshes/Untitled.dae`（纯视觉网格），
生成 `src/robocon_gazebo/models/robocon_ground_full/model.sdf`（带完整碰撞体）。

**什么时候需要跑**：你换了 DAE 视觉网格、或者改了 DAE 里的结构。
**什么时候不要跑**：你刚在场地编辑器里手动调过尺寸 —— 生成器会用代码里的默认值覆盖你的改动。

> 生成器里已同步当前场地的实际数值（4 段地面隔板 + 2 段 L1 围栏，碰撞与视觉各自独立），
> 重跑输出与现有 `model.sdf` 逐字节一致。但你在编辑器里再改之后，记得把新数值同步回生成器。

### `strip_fixed_tower_tops.py` — gen 的前置步骤

只在换了 DAE 视觉网格时才需要：

```bash
/usr/bin/python3 tool/scripts/strip_fixed_tower_tops.py   # 先剥掉固定塔顶节点
/usr/bin/python3 tool/scripts/gen_full_collision.py       # 再生成碰撞
```

### `run_mujoco.sh` / `selftest_mujoco.py` — MuJoCo 机器狗

这条链路和 Gazebo 场地**完全独立**，用的是机器狗 + 强化学习策略。

```bash
./tool/scripts/run_mujoco.sh                    # 可视化 + 键盘控制
./tool/scripts/run_mujoco.sh --no-policy        # 只 PD 站立，不跑策略
./tool/scripts/run_mujoco.sh --duration 10      # 跑 10 秒后退出

# 无界面自检
~/.venvs/robocon_mujoco/bin/python tool/scripts/selftest_mujoco.py --seconds 3
```

依赖专用 venv `~/.venvs/robocon_mujoco`（mujoco + onnxruntime）。缺了会报错并提示重建命令。

---

## `gazebo_models/` 是什么

上游仓库 [jimi4-lei/gazebo_models](https://github.com/jimi4-lei/gazebo_models) 的完整克隆，
是南华大学衡山战队发布的原始场地资源。

**它是只读的，不要改里面任何文件**。原因：

- `gen_full_collision.py` 从这里读 DAE 源文件
- `setup_env.sh` / `run_gazebo.sh` 把这里加进 `GAZEBO_MODEL_PATH`
- `run_gazebo.sh --legacy` 直接调这里的 `run_gazebo.sh`
- MuJoCo 演示从这里读模型和策略

你要改场地，改的是 `src/robocon_gazebo/models/robocon_ground_full/model.sdf`
（或用编辑器改），不是这里。

里面有些脚本（`clip_ground_edges.py` 等）硬编码了原作者机器的路径
`/home/uwvwko/桌面/rc/...`，在本机跑不了 —— 正常，它们是历史遗留，不用管。

---

## 常见问题

**Q: 改了脚本但行为没变？**
A: `field_editor_server.py` 会生成 `__pycache__`。启动脚本已加 `-B` 禁用字节码缓存，
如果还有问题，手动 `rm -rf tool/scripts/__pycache__` 再试。

**Q: 点"应用"之后场地变灰了？**
A: 打开 `tool/scripts/field_editor_server.py`，检查 `read_state()` 是否按文档原始顺序
遍历 link 子节点（不能按 kind 分组）、`render_item()` 是否原样输出 `materialRaw`。
这三条约束在文件注释里写明了。

**Q: `run_gazebo.sh` 报"尚未构建 ROS 包"？**
A: 先在工作区根执行：
```bash
source /opt/ros/humble/setup.bash
PATH=/usr/bin:$PATH colcon build --symlink-install --packages-select robocon_gazebo \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

**Q: 想加新的场地元素？**
A: 用编辑器加（点"新增节点"），或者在 `model.sdf` 里手工加，然后**把新数值同步进
`gen_full_collision.py`**，否则下次重跑生成器会丢掉。
