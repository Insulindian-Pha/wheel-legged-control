import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('five_bar_link')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'FiveBarLink.urdf')

    # 读取URDF文件并替换package://five_bar_link/为绝对路径
    # 这样可以让Gazebo找到mesh文件,同时保持URDF中的package://格式
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # 移除XML编码声明以避免spawn_entity.py解析错误
    if robot_desc.startswith('<?xml'):
        robot_desc = robot_desc.split('?>', 1)[1].lstrip()
    
    # 将package://five_bar_link/替换为包共享目录的绝对路径
    robot_desc = robot_desc.replace('package://five_bar_link/', pkg_share + '/')

    # 启动Gazebo并加载ROS插件
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 静态变换发布器 (base_link到base_footprint)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    )

    # 在Gazebo中生成机器人
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'five_bar_link_wheel',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_cmd,
        robot_state_publisher_node,
        static_tf_node,
        spawn_entity_node
    ])

