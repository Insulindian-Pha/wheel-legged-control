#!/usr/bin/env python3
"""
Gazebo with ROS2 Control Launch File
启动Gazebo仿真环境并集成ros2_control力矩控制器
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

    # 使用subprocess处理xacro文件生成URDF,并保留package:// mesh路径供Foxglove加载
    robot_desc = subprocess.run(
        ['xacro', xacro_file, 'use_gazebo:=true'],
        capture_output=True,
        text=True,
        check=True
    ).stdout

    # 启动Gazebo(使用单个gazebo命令,加载ROS插件)
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True  # 使用Gazebo仿真时间
        }]
        # 直接订阅 /joint_states，不需要 remapping
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'source_list': ['joint_states'],  # 订阅来自ros2_control的joint_states
            'use_sim_time': True
        }]

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

    # Joint Group Effort Controller Spawner - 力矩控制器
    joint_group_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_effort_controller',
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

    # 然后启动joint_group_effort_controller
    delayed_joint_group_effort_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_group_effort_controller_spawner]
        )
    )

    # Joint Torque Controller Node - 读取关节状态并发布力矩命令
    joint_torque_controller_node = Node(
        package='joint_torque_controller',
        executable='joint_torque_controller',
        name='joint_torque_controller',
        parameters=[{
            'joint_names': [
                'Left_front_joint',
                'Left_rear_joint',
                'Left_Wheel_joint',
                'Right_front_joint',
                'Right_rear_joint',
                'Right_Wheel_joint'
            ],
            'joint_state_topic': '/joint_states',
            'torque_command_topic': '/joint_group_effort_controller/commands',
            'controller_name': 'joint_group_effort_controller',
            'publish_rate': 500.0,
            'max_torque': 20.0,
            'use_sim_time': True
        }]
    )

    # Control Converter Node - 将control_input_msgs转换为关节力矩命令
    control_converter_node = Node(
        package='control_converter',
        executable='control_converter',
        name='control_converter',
        parameters=[{
            'max_torque_wheel': 5.0,   # 轮子关节最大力矩 (Nm)
            'max_torque_front': 20.0,  # 前关节最大力矩 (Nm)
            'max_torque_rear': 20.0,   # 后关节最大力矩 (Nm)
            'control_input_topic': 'control_input',
            'torque_command_topic': '/joint_torque_controller/torque_commands',
            'publish_rate': 20.0,
            'use_sim_time': True
        }]
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
        delayed_joint_group_effort_controller,
        
        # 控制转换节点
        control_converter_node,
        
        # 关节力矩控制器节点
        joint_torque_controller_node,
    ])


# 使用说明:
# 1. 启动仿真:
#    ros2 launch bringup gazebo_ros2_control.launch.py
#
# 2. 使用键盘控制机器人:
#    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

