import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('cod_2026_balance')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'COD_2026_Balance_2_0.urdf')

    # Read URDF file and replace package://cod_2026_balance/ with absolute path
    # This allows Gazebo to find mesh files while keeping package:// format in URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Remove XML encoding declaration to avoid spawn_entity.py parsing errors
    if robot_desc.startswith('<?xml'):
        robot_desc = robot_desc.split('?>', 1)[1].lstrip()
    
    # Replace package://cod_2026_balance/ with absolute path to package share directory
    robot_desc = robot_desc.replace('package://cod_2026_balance/', pkg_share + '/')

    # Start Gazebo with ROS plugins
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # Robot State Publisher node
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

    # Static transform publisher (base_link to base_footprint)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    )

    # Spawn robot in Gazebo (using gazebo_ros spawn service)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'cod_2026_balance',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_cmd,
        robot_state_publisher_node,
        static_tf_node,
        spawn_entity_node
    ])
