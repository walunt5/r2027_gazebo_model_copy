# ROBOCON 2027 场地与底盘仿真

本仓库提供 ROBOCON 2027 场地的 Gazebo Classic 仿真、天空块和地基块摆放工具，以及一个可在场地中运行的麦克纳姆底盘机器人。项目由深圳职业技术大学 Techx 战队整理；场地资源基于[南华大学衡山丌战队的 gazebo_models](https://github.com/jimi4-lei/gazebo_models)修改,优化了碰撞模型。

## 先看懂工程

| 部分 | 源码位置 | 作用 | 启动入口 |
| --- | --- | --- | --- |
| 场地 | [`src/robocon_gazebo/`](src/robocon_gazebo/) | 完整碰撞场地、天空块/地基块模型、两个摆放窗口 | `robocon_gazebo.launch.py` |
| 底盘机器人 | [`src/r2027_robot_description/`](src/r2027_robot_description/) | Xacro 机器人描述、平面驱动插件、物理麦轮控制器、速度窗口 | `robocon_robot.launch.py` |
| 模型与辅助工具 | [`tool/`](tool/) | 上游场地资源、场地编辑器、碰撞生成脚本、独立 MuJoCo 示例 | 按需运行，见[工具说明](tool/README.md) |

启动顺序是**场地 → 机器人**。场地 launch 启动 Gazebo 服务端和图形客户端，并默认打开天空块、地基块摆放窗口；机器人 launch 把底盘实体加入已经运行的 Gazebo，不会再启动第二个 Gazebo。只看场地时不必启动机器人。两种功能分别属于上表中的两个 ROS 包。

## 环境与安装

建议使用 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11。需要 `colcon`、`rosdep`，以及两个 ROS 包各自 `package.xml` 中声明的依赖。天空块和地基块窗口依赖 Python Tk；底盘速度窗口依赖 PyQt5。MuJoCo 示例使用独立环境，**不是**运行 Gazebo 场地的前置条件。

下载本仓库后进入其根目录。在**尚未自动加载 ROS 2 Humble** 的终端，先运行：

```bash
source /opt/ros/humble/setup.bash
```

首次安装依赖可使用（需要已完成本机的 `rosdep` 初始化）：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

然后构建并加载工作区：

```bash
colcon build
source install/setup.bash
```

以后每个新终端都要加载本工作区的 `install/setup.bash`；若该终端未自动加载 ROS，也要先加载 `/opt/ros/humble/setup.bash`。构建产物保存在 `build/`、`install/`、`log/`，它们不应提交到 Git。`tool/scripts/setup_env.sh` 可额外配置模型搜索路径和清理代理；常规的两条 ROS 启动命令不要求使用它。

## 快速启动：两个终端

### 终端 A：场地

```bash
source install/setup.bash
ros2 launch robocon_gazebo robocon_gazebo.launch.py
```

默认加载完整碰撞场地，启动 Gazebo 图形窗口、天空块摆放窗口和地基块摆放窗口。天空块窗口会在 Gazebo 服务就绪后生成第一组天空块；地基块窗口只在用户选择颜色和区域并点击生成后才创建地基块。场地提供 `/clock` 和 `/spawn_entity` 等 Gazebo ROS 服务。

### 终端 B：底盘机器人

保持终端 A 运行，在另一终端执行：

```bash
source install/setup.bash
ros2 launch r2027_robot_description robocon_robot.launch.py
```

默认在蓝方启动区生成名为 `r2027_robot` 的底盘机器人，位置为 `x=-5.1 m, y=-4.4 m, z=0.15 m, yaw=0`；同时启动 RViz 和底盘速度窗口。机器人启动命令会等待 Gazebo 的 `/spawn_entity` 服务，默认最多等待 120 秒。场地中心有障碍物，避免使用 `x:=0 y:=0` 出生。

退出时在两个终端分别按 `Ctrl+C`。重新运行终端 B 前，应确认旧机器人实体已经随 Gazebo 退出，或使用不同的 `entity` 名称。

## 启动参数

### 场地 `robocon_gazebo.launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 启动 Gazebo 3D 客户端；设为 `false` 时两个摆放窗口也不会启动 |
| `placement_gui` | `true` | 启动天空块摆放窗口，窗口会自动生成第一组天空块 |
| `tower_base_gui` | `true` | 启动地基块摆放窗口；窗口本身不自动生成地基块 |
| `paused` | `false` | 启动时暂停 Gazebo 物理仿真 |
| `verbose` | `false` | 输出更详细的 Gazebo 服务端日志 |
| `world` | 完整碰撞场地 | 传入其他 `.world` 文件以替换场地 |

`placement_gui` 和 `tower_base_gui` 只有在 `gui:=true` 时才生效。常见组合：

```bash
# 仅启动 Gazebo 3D 场地，不打开两个摆放窗口
ros2 launch robocon_gazebo robocon_gazebo.launch.py placement_gui:=false tower_base_gui:=false

# 无图形界面的 Gazebo 服务端；两个摆放窗口也会关闭
ros2 launch robocon_gazebo robocon_gazebo.launch.py gui:=false
```

### 机器人 `robocon_robot.launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `drive` | `planar` | `planar`：平面全向驱动，可横移；`physics`：`ros2_control` 麦轮模型 |
| `x`、`y`、`z`、`yaw` | `-5.1`、`-4.4`、`0.15`、`0.0` | 机器人在 Gazebo 中的出生位姿，位置单位 m、角度单位 rad |
| `entity` | `r2027_robot` | Gazebo 实体名，同一世界内必须唯一 |
| `spawn_timeout` | `120.0` | 等待 `/spawn_entity` 服务的秒数 |
| `rviz` | `true` | 启动 RViz 机器人视图 |
| `chassis_gui` | `true` | 启动底盘速度窗口 |
| `cmd_timeout` | `0.5` | `/cmd_vel` 停止输入后自动归零的秒数 |

例如，无 RViz 和底盘窗口运行物理麦轮模式：

```bash
ros2 launch r2027_robot_description robocon_robot.launch.py \
  drive:=physics rviz:=false chassis_gui:=false
```

`planar` 模式使用本仓库的重力兼容平面驱动插件，支持前后、横移和转向，并保留竖直方向的重力与碰撞响应。`physics` 模式使用四个普通圆柱轮的 `ros2_control` 模型；Gazebo ODE 不能准确模拟麦克纳姆辊子的侧向接触，因此横移可能漂移。两种模式都通过 `/cmd_vel` 接收 `geometry_msgs/msg/Twist`。

## 常用操作

### 控制底盘

底盘速度窗口提供前后、左右和旋转控制；按住方向按钮时以 20 Hz 发布，松开时发送零速度。也可以从终端持续发送速度：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}"
```

`cmd_vel_watchdog` 在输入静默超过 `cmd_timeout` 后发送零速度。机器人发布 `/odom` 和 `/tf`；物理麦轮模式还会发布四轮 `/joint_states`。有关底盘结构、坐标系和控制接口，见[机器人说明](src/r2027_robot_description/README.md)。

### 摆放天空块

天空块窗口支持随机选择或指定四种规则布局、预览 5×5 棋盘、清空并重新生成。每组有 12 个独立动态实例，模型位于 [`robocon_tower_top/model.sdf`](src/robocon_gazebo/models/robocon_tower_top/model.sdf)。块的局部 `+x` 是红半面、`-x` 是蓝半面；布局通过实例旋转实现。窗口快捷键：`1`–`4` 选择布局，`R` 随机，`C` 清空。

只打印可复现布局、不打开窗口：

```bash
ros2 run robocon_gazebo tower_top_placement_gui.py --print-plan --layout 0 --seed 2027
```

`--preview-only` 会打开预览窗口，但不会自动生成第一组天空块。上述命令需先 `source install/setup.bash`。

### 摆放地基块

地基块窗口可以在四个预设区域生成红、蓝方块；默认不自动生成。区域 1/2 位于地面储存区，3/4 位于 L1 传递区。每格最多叠两块，窗口会读取 Gazebo 已有实例以恢复计数。模型分别位于 [`robocon_tower_base_red`](src/robocon_gazebo/models/robocon_tower_base_red/model.sdf) 和 [`robocon_tower_base_blue`](src/robocon_gazebo/models/robocon_tower_base_blue/model.sdf)。也可以用 `ros2 run robocon_gazebo tower_base_placement_gui.py` 单独启动窗口。

如果需要在任意位置放置方块，可直接调用 Gazebo 服务。例如在地面生成红色方块，`z=0.175` 是方块中心高度：

```bash
ros2 run gazebo_ros spawn_entity.py \
  -file src/robocon_gazebo/models/robocon_tower_base_red/model.sdf \
  -entity tower_base_red_01 -x -4.0 -y 1.0 -z 0.175
```

同一 Gazebo 世界中，每次调用都应使用新的 `-entity` 名称。L1/L2 台面上方的方块中心高度分别约为 `z=0.775`、`z=1.075`。

## 场地编辑与辅助工具

从仓库根目录运行 `./tool/scripts/run_field_editor.sh`，再打开 `http://127.0.0.1:8790`。编辑器支持拖动、数值调整、预览和撤销；点击“应用到工程”会写入场地 SDF、备份原文件，并执行 SDF 与构建检查。要改端口，可传入数字，例如 `./tool/scripts/run_field_editor.sh 9000`。

完整碰撞场地的权威生成脚本是 [`gen_full_collision.py`](tool/scripts/gen_full_collision.py)。修改原始 DAE 视觉网格后，按[工具说明](tool/README.md)中的顺序先运行 `strip_fixed_tower_tops.py`，再重新生成 SDF。重新生成会覆盖场地编辑器的部分手工几何调整，需先核对生成脚本中的参数。编辑器备份存放在 `.cache/sdf_backup/`，不提交 Git。

`./tool/scripts/run_gazebo.sh` 是可选包装脚本：默认启动完整场地，`--headless` 启动无界面服务端，`--legacy` 启动上游旧场地。MuJoCo 机器狗示例与 ROS/Gazebo 流程独立，运行方法见[工具说明](tool/README.md)。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `ros2: command not found` 或找不到 ROS 包 | 先加载 `/opt/ros/humble/setup.bash`，构建后再加载当前仓库的 `install/setup.bash`；用 `ros2 pkg prefix robocon_gazebo` 确认路径属于当前仓库 |
| Gazebo 已启动，但没有天空块或地基块窗口 | 确认没有传入 `gui:=false`、`placement_gui:=false`、`tower_base_gui:=false`；检查启动终端中两个 `tower_*_placement_gui.py` 进程是否报错 |
| GUI 报 `FileNotFoundError`，路径指向 `install/.../lib/models` | 这是旧脚本的模型路径错误；更新源码并在当前仓库重新执行 `colcon build`，再加载新的 `install/setup.bash` |
| 窗口无法打开或显示异常 | 确认使用有桌面会话的终端，`DISPLAY` 已设置，Python Tk 和 PyQt5 已安装；Gazebo 3D 渲染问题可尝试 `LIBGL_ALWAYS_SOFTWARE=1` |
| 机器人等待 `/spawn_entity` 超时 | 先启动场地并确认 `gzserver` 正在运行，再启动机器人；必要时增大 `spawn_timeout` |
| Gazebo 模型资源加载慢或 `libcurl` 报错 | 可检查网络/代理；本仓库所需的本地模型已随项目提供，`tool/scripts/setup_env.sh --no-proxy` 可清理当前终端代理 |
| `physics` 模式横移不准 | 普通圆柱轮与 ODE 接触模型的限制；需要稳定横移时使用默认 `drive:=planar` |

## 发布与许可

`build/`、`install/`、`log/`、`.cache/` 等本地文件已列入 `.gitignore`。发布前应检查准备提交的文件和大型模型资源，并为本仓库代码及所含上游资源确认适用的许可证和署名。当前两个 ROS 包的 `package.xml` 声明 `Apache-2.0`；仓库根目录尚无独立的许可证文件，发布者应核对上游资源许可后补齐。
