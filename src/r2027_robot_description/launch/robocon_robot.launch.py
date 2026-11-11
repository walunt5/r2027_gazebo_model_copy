"""Load the mecanum robot into a running Gazebo world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('r2027_robot_description')

    xacro_file = os.path.join(
        description_share, 'urdf', 'r2027_robot.urdf.xacro')
    rviz_config = os.path.join(description_share, 'rviz', 'robot.rviz')

    declared_args = [
        DeclareLaunchArgument(
            'drive', default_value='planar',
            choices=['planar', 'physics'],
            description='planar: kinematic omni-drive (strafe works); '
                        'physics: ros2_control mecanum wheels (no roller friction in ODE).'),
        DeclareLaunchArgument(
            'cmd_timeout', default_value='0.5',
            description='Seconds of /cmd_vel silence before the watchdog '
                        'zeros a non-zero command (0.5 s matches the old '
                        'ros2_control controller timeout).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the robot display configuration.'),
        DeclareLaunchArgument(
            'chassis_gui', default_value='true',
            description='Start the mecanum chassis velocity control window.'),
        DeclareLaunchArgument(
            'entity', default_value='r2027_robot',
            description='Name of the robot entity created in Gazebo.'),
        DeclareLaunchArgument(
            'spawn_timeout', default_value='120.0',
            description='Seconds to wait for the Gazebo /spawn_entity service.'),
        DeclareLaunchArgument(
            'x', default_value='-5.1',
            description='Robot spawn x position [m].'),
        DeclareLaunchArgument(
            'y', default_value='-4.4',
            description='Robot spawn y position [m].'),
        DeclareLaunchArgument(
            'z', default_value='0.15',
            description='Robot spawn z position [m].'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Robot spawn yaw angle [rad].'),
    ]

    planar_xacro_arg = PythonExpression(
        ["'planar_drive:=true' if '", LaunchConfiguration('drive'),
         "' == 'planar' else 'planar_drive:=false'"])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' ', planar_xacro_arg]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', LaunchConfiguration('entity'),
            '-timeout', LaunchConfiguration('spawn_timeout'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-package_to_model',
        ],
    )

    load_joint_state_broadcaster = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('drive'), "' == 'physics'"])),
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    chassis_gui = Node(
        package='r2027_robot_description',
        executable='chassis_gui.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_rate': 20.0,
            'linear_limit': 1.0,
            'angular_limit': 1.5,
        }],
        condition=IfCondition(LaunchConfiguration('chassis_gui')),
    )

    load_mecanum_controller = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('drive'), "' == 'physics'"])),
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    # Expose the standard unstamped Twist command and common output topics.
    cmd_vel_relay = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('drive'), "' == 'physics'"])),
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/mecanum_drive_controller/reference_unstamped',
            'lazy': False,
        }],
    )
    odom_relay = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('drive'), "' == 'physics'"])),
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',
        parameters=[{
            'input_topic': '/mecanum_drive_controller/odometry',
            'output_topic': '/odom',
            'lazy': False,
        }],
    )
    tf_relay = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('drive'), "' == 'physics'"])),
        package='topic_tools',
        executable='relay',
        name='odom_tf_relay',
        output='screen',
        parameters=[{
            'input_topic': '/mecanum_drive_controller/tf_odometry',
            'output_topic': '/tf',
            'lazy': False,
        }],
    )

    # Safety: zero /cmd_vel if the commanding node (e.g. teleop) goes
    # silent while the last command was non-zero (planar_move has no
    # built-in cmd_vel timeout).
    cmd_vel_watchdog = Node(
        package='r2027_robot_description',
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{
            'timeout': ParameterValue(LaunchConfiguration('cmd_timeout'), value_type=float),
            'rate': 20.0,
        }],
    )

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
            rviz,
            spawn_robot,
            cmd_vel_watchdog,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[chassis_gui, load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_mecanum_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_mecanum_controller,
                    on_exit=[cmd_vel_relay, odom_relay, tf_relay],
                )
            ),
        ]
    )
