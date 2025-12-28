#!/usr/bin/env python3
"""
Gazebo with ROS2 Control Launch File
启动Gazebo仿真环境并集成ros2_control控制器
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('cod_2026_balance')
    bringup_share = get_package_share_directory('bringup')
    
    # xacro文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'cod_balance_robot.xacro')

    # 使用subprocess处理xacro文件生成URDF,并设置use_gazebo参数为true
    robot_desc = subprocess.run(
        ['xacro', xacro_file, 'use_gazebo:=true'],
        capture_output=True,
        text=True,
        check=True
    ).stdout
    
    # 替换package://路径为file://绝对路径,使Gazebo和RViz都能找到mesh文件
    robot_desc = robot_desc.replace('package://cod_2026_balance/', 'file://' + pkg_share + '/')

    # 启动Gazebo(使用单个gazebo命令,加载ROS插件)
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Joint State Publisher节点 - 合并ros2_control的关节状态与其他关节状态
    # 订阅 /joint_states (来自ros2_control的joint_state_broadcaster), 合并后发布到 /joint_states_merged
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'source_list': ['joint_states'],  # 订阅来自ros2_control的joint_states
            'use_sim_time': True
        }],
        remappings=[
            ('joint_states', 'joint_states_merged')  # 发布合并后的状态到不同的topic
        ]
    )

    # Robot State Publisher节点 - 发布机器人描述和TF变换
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True  # 使用Gazebo仿真时间
        }],
        remappings=[
            ('joint_states', 'joint_states_merged')  # 订阅合并后的关节状态
        ]
    )

    # 静态TF发布器 - base_link到base_footprint
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    )

    # 在Gazebo中生成机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'cod_2026_balance',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'  # 在地面上方0.5米生成
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner - 发布关节状态
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Diff Drive Controller Spawner - 差速驱动控制器
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 延迟启动控制器,等待Gazebo和机器人完全加载
    # 首先启动joint_state_broadcaster
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # 然后启动diff_drive_controller
    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner]
        )
    )

    return LaunchDescription([
        # 启动Gazebo
        start_gazebo_cmd,
        
        # 发布机器人状态
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf_node,
        
        # 在Gazebo中生成机器人
        spawn_entity_node,
        
        # 延迟启动控制器
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
    ])


# 使用说明:
# 1. 启动仿真:
#    ros2 launch bringup gazebo_ros2_control.launch.py
#
# 2. 使用键盘控制机器人:
#    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true


