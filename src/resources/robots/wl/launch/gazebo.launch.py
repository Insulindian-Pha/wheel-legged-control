import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('wl')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'wl.urdf')

    # Read URDF file and replace package://wl/ with absolute path
    # This allows Gazebo to find mesh files while keeping package:// format in URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Replace package://wl/ with absolute path to package share directory
    robot_desc = robot_desc.replace('package://wl/', pkg_share + '/')

    # Path to world file
    world_file = os.path.join(pkg_share, 'worlds', 'default.sdf')

    # Include Gazebo launch file with world file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_file}.items()
    )

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
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wl',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # Fake joint calibration publisher (runs after spawn completes)
    calibration_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', '{data: true}', '--once', '--no-wait'],
        output='screen'
    )

    # Register event handler to run calibration publisher after spawn completes
    calibration_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[calibration_publisher]
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        static_tf_node,
        spawn_entity_node,
        calibration_event_handler
    ])
