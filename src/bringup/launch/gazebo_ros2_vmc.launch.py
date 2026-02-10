#!/usr/bin/env python3
"""
Gazebo with ROS2 Control VMC Launch File
启动Gazebo仿真环境，机器人固定在半空中，使用VMC控制器
用于观察VMC控制效果
"""

import os
import re
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
    
    # 按照博客方法：创建world link，然后用fixed关节将base_link固定到world
    # 在robot标签开头添加world link和固定关节
    world_fixed_xml = '''  <!-- World link to fix robot in air for VMC observation -->
  <link name="world"/>
  
  <joint name="world_base_fixed_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  
'''
    
    # 在</robot>标签前添加Gazebo配置，确保world link是静态的
    gazebo_fixed_xml = '''  <!-- Gazebo plugin to make world link static -->
  <gazebo reference="world">
    <static>true</static>
  </gazebo>
'''
    
    # 在robot标签后插入world link（使其成为根link）
    robot_desc = re.sub(r'(<robot[^>]*>)', r'\1\n' + world_fixed_xml, robot_desc, count=1)
    
    # 在</robot>标签前插入Gazebo配置
    robot_desc = robot_desc.replace('</robot>', gazebo_fixed_xml + '</robot>')

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

    # 在Gazebo中生成机器人模型 - 固定在半空中
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'cod_2026_balance',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '1.0'  # 在半空中1.0米高度生成
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

    # VMC Controller Spawner - VMC控制器
    vmc_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'vmc_controller',
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

    # 然后启动vmc_controller
    delayed_vmc_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[vmc_controller_spawner]
        )
    )

    return LaunchDescription([
        # 启动Gazebo
        start_gazebo_cmd,
        
        # 发布机器人状态
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf_node,
        
        # 在Gazebo中生成机器人（固定在半空中）
        spawn_entity_node,
        
        # 延迟启动控制器
        delayed_joint_state_broadcaster,
        delayed_vmc_controller,
    ])


# 使用说明:
# 1. 启动仿真:
#    ros2 launch bringup gazebo_ros2_vmc.launch.py
#
# 2. 发布VMC力命令:
#    ros2 topic pub /vmc_controller/force_command std_msgs/msg/Float64MultiArray "{data: [left_F0, left_Tp, right_F0, right_Tp]}"
#
# 3. 机器人固定在半空中，可以观察VMC控制效果，不受重力影响

