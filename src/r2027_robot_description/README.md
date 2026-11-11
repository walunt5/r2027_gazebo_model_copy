# ROBOCON 2027 麦克纳姆底盘机器人描述

本包提供底盘 Xacro 描述、Gazebo `ros2_control` 配置，以及把机器人加载到已运行 ROBOCON 场地中的 launch。

## 构建

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 启动

场地 Gazebo 和机器人使用两个 launch，分别放在两个终端中运行。

终端 A，先启动场地：

```bash
source tool/scripts/setup_env.sh --no-proxy
ros2 launch robocon_gazebo robocon_gazebo.launch.py
```

终端 B，把整机加载到正在运行的场地：

```bash
source tool/scripts/setup_env.sh --no-proxy
ros2 launch r2027_robot_description robocon_robot.launch.py \
  x:=-5.1 y:=-4.4 z:=0.15 yaw:=0.0
```

场地 launch 只启动 Gazebo，不加载机器人；机器人 launch 不启动 Gazebo，只生成底盘机器人，并启动描述与底盘控制。机器人 launch 会等待 Gazebo 的 `/spawn_entity` 服务，默认最多等待 `120 s`，可用 `spawn_timeout` 修改。

默认出生点为 `x=-5.1 m, y=-4.4 m, z=0.15 m, yaw=0`，可通过 `x/y/z/yaw` 覆盖，也可用 `entity` 修改实体名。RViz 默认打开 RobotModel 和 TF 显示，可用 `rviz:=false` 关闭。底盘速度窗口默认打开，可用 `chassis_gui:=false` 关闭。

场地参数仍由第一个 launch 管理：`gui:=false` 无界面启动，`paused:=true` 暂停启动，`world:=...` 替换世界文件。

不要使用 `x:=0 y:=0` 作为出生点。赛道中心包含中央隔板、五色石基座和 L1/L2 碰撞层，机器人会与重叠碰撞体发生严重弹射并飞出场景。

## 底盘参数

底盘本体质量由 `base_mass` 控制，当前为 `10.0 kg`；车体三个惯量分量根据长、宽、高和质量自动计算。

## 底盘速度窗口

运行整机 launch 后，底盘窗口默认打开。它提供：

- 前后速度 `linear.x`、横向速度 `linear.y`、旋转速度 `angular.z` 三组滑块和精确数值输入；
- 前进、后退、左移、右移、左转、右转快捷方向；
- 方向按钮采用按住控制：按住时以 `20 Hz` 发布 `/cmd_vel`，松开立即发送零速度；
- 空闲时不发送 `/cmd_vel`，窗口失去焦点、点击“停止 / 全部归零”或关闭窗口时也会立即停车。

只启动底盘窗口：

```bash
ros2 run r2027_robot_description chassis_gui.py
```

## 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 标准底盘速度输入 |
| `/odom` | `nav_msgs/msg/Odometry` | 四轮里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | physics 模式下的四轮关节反馈 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 底盘关节、`odom -> base_link` 和地面投影坐标系 |

`base_link` 是 URDF 根坐标系。`footprint_base_link` 是其地面投影，位于 `base_link` 下 `0.15 m` 处，不包含质量或碰撞体。

轮子为普通圆柱碰撞体的麦克纳姆近似模型，半径 `0.10 m`，轮子中心位于 `x=±0.18 m`、`y=±0.22 m`。控制器执行正确的麦克纳姆运动学；Gazebo 中普通圆柱接触的侧向摩擦精度有限。

`planar` 模式使用本包提供的重力兼容平面驱动插件。它继续约束底盘的水平速度和偏航速度，但保留 Gazebo 产生的竖直速度、横滚和俯仰角速度，因此底盘会正常受重力、从高台自由落体，并在落地后保持全向控制。`/odom` 的位置和速度也包含实际竖直分量。

## 控制测试

以 20 Hz 持续发布，避免超过 `0.5 s` 控制超时：

```bash
# 前进
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 向左横移
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"

# 逆时针旋转
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}"

# 斜向移动
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.3}, angular: {z: 0.2}}"
```

检查控制器和接口：

```bash
ros2 control list_controllers
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link footprint_base_link
```

`planar` 模式由平面驱动插件控制；`physics` 模式激活 `joint_state_broadcaster` 和 `mecanum_drive_controller`。
