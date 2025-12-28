import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('cod_2026_balance')
    
    # Path to MJCF file
    mjcf_file = os.path.join(pkg_share, 'MJCF', 'COD-2026RoboMaster-Balance.xml')
    
    # Declare launch argument for MuJoCo command (optional customization)
    mujoco_cmd_arg = DeclareLaunchArgument(
        'mujoco_cmd',
        default_value='python3 -m mujoco.viewer',
        description='MuJoCo command to run (default: python3 -m mujoco.viewer)'
    )
    
    # Create MuJoCo process using OpaqueFunction to handle LaunchConfiguration
    def create_mujoco_process(context):
        cmd_str = context.launch_configurations.get('mujoco_cmd', 'python3 -m mujoco.viewer')
        full_cmd = f'{cmd_str} --mjcf {mjcf_file}'
        return [ExecuteProcess(
            cmd=['bash', '-c', full_cmd],
            output='screen',
            cwd=os.path.dirname(mjcf_file)  # Set working directory to MJCF folder so mesh paths work
        )]
    
    mujoco_process = OpaqueFunction(function=create_mujoco_process)
    
    return LaunchDescription([
        mujoco_cmd_arg,
        mujoco_process
    ])

