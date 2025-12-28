import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('cod_2026_balance')

    # Get xacro file path (use the main xacro file that includes others)
    xacro_file = os.path.join(pkg_share, 'urdf', 'cod_balance_robot.xacro')

    # Process xacro file to generate URDF
    # xacro will automatically handle the include and resolve package:// paths
    robot_desc = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True,
        text=True,
        check=True
    ).stdout
    
    robot_desc = robot_desc.replace('package://cod_2026_balance/', 'file://' + pkg_share + '/')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ])

