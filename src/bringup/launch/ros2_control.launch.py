from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('cod_2026_balance'),
                'urdf',
                'cod_balance_robot.xacro'
            ])
        ]),
        value_type=str
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'config',
                'cod_balance_controllers.yaml'
            ])
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller']
    )

    # Joint State Publisher to merge joint states from ros2_control with default states for other joints
    # It subscribes to /joint_states (from joint_state_broadcaster) and merges with default states (0)
    # for all other joints defined in URDF, then publishes merged states to /joint_states_merged
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'source_list': ['joint_states']  # Subscribe to joint_states from ros2_control
        }],
        remappings=[
            ('joint_states', 'joint_states_merged')  # Publish merged states to a different topic
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }],
        remappings=[
            ('joint_states', 'joint_states_merged')  # Subscribe to merged joint states
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        joint_state_publisher_node
    ])

# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

