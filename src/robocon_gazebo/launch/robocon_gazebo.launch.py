"""只启动 ROBOCON 2027 场地 (Gazebo Classic 11 + ROS 2 Humble).

用法:
    ros2 launch robocon_gazebo robocon_gazebo.launch.py
    ros2 launch robocon_gazebo robocon_gazebo.launch.py gui:=false

机器人通过 r2027_robot_description/robocon_robot.launch.py 单独加载。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.condition import Condition
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class AndCondition(Condition):
    """Humble does not export launch.conditions.AndCondition."""

    def __init__(self, *conditions):
        super().__init__(
            predicate=lambda context: all(
                condition.evaluate(context) for condition in conditions
            )
        )


def generate_launch_description():
    share = get_package_share_directory('robocon_gazebo')
    models_dir = os.path.join(share, 'models')
    default_world = os.path.join(share, 'worlds', 'robocon_track_full.world')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # 模型查找路径: 本包安装的模型 + 原有环境 + 用户 ~/.gazebo/models
    model_path_parts = [
        models_dir,
        os.path.expanduser('~/.gazebo/models'),
    ]
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_path_parts.append(os.environ['GAZEBO_MODEL_PATH'])
    model_path = os.pathsep.join(model_path_parts)

    declared_args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 gzclient 图形界面'),
        DeclareLaunchArgument(
            'placement_gui', default_value='true',
            description='是否启动塔顶随机摆放控制 GUI（同时自动生成首组塔顶）'),
        DeclareLaunchArgument(
            'tower_base_gui', default_value='true',
            description='是否启动地基块按颜色与区域生成 GUI（窗口打开时不自动生成）'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='启动后暂停仿真'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='gzserver 输出详细日志'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='world 文件 (默认完整碰撞版场地; 旧版用 worlds/robocon_track_ros.world)'),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': LaunchConfiguration('verbose'),
            'pause': LaunchConfiguration('paused'),
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    placement_gui = Node(
        package='robocon_gazebo',
        executable='tower_top_placement_gui.py',
        name='rc2027_tower_top_placement',
        output='screen',
        condition=AndCondition(
            IfCondition(LaunchConfiguration('gui')),
            IfCondition(LaunchConfiguration('placement_gui')),
        ),
    )

    tower_base_gui = Node(
        package='robocon_gazebo',
        executable='tower_base_placement_gui.py',
        name='rc2027_tower_base_placement',
        output='screen',
        condition=AndCondition(
            IfCondition(LaunchConfiguration('gui')),
            IfCondition(LaunchConfiguration('tower_base_gui')),
        ),
    )

    return LaunchDescription(
        [SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)]
        + declared_args
        + [gzserver, gzclient, placement_gui, tower_base_gui]
    )
