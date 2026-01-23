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

    joint_group_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller']
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

    # Control Converter Node - converts control_input_msgs to joint torque commands
    control_converter_node = Node(
        package='control_converter',
        executable='control_converter',
        name='control_converter',
        parameters=[{
            'max_torque_wheel': 10.0,   # Maximum torque for wheel joints (Nm)
            'max_torque_front': 10.0,  # Maximum torque for front joints (Nm)
            'max_torque_rear': 10.0,   # Maximum torque for rear joints (Nm)
            'control_input_topic': 'control_input',
            'torque_command_topic': '/joint_torque_controller/torque_commands',
            'publish_rate': 50.0
        }]
    )

    # Joint Torque Controller Node - reads joint states and publishes torque commands
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
            'publish_rate': 50.0,
            'max_torque': 30.0
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        joint_group_effort_controller_spawner,
        joint_state_publisher_node,
        control_converter_node,
        joint_torque_controller_node
    ])

# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

