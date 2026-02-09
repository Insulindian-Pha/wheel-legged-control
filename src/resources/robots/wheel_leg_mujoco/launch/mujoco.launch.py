import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('wheel_leg_mujoco')
    
    # Path to MJCF file
    mjcf_file = os.path.join(pkg_share, 'MJCF', 'env.xml')
    mjcf_dir = os.path.join(pkg_share, 'MJCF')
    wheel_leg_mujoco_dir = pkg_share
    
    # Declare launch argument for MuJoCo command (optional customization)
    mujoco_cmd_arg = DeclareLaunchArgument(
        'mujoco_cmd',
        default_value='python3 -m mujoco.viewer',
        description='MuJoCo command to run (default: python3 -m mujoco.viewer)'
    )
    
    # Declare launch argument for simulation mode
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='viewer',
        choices=['viewer', 'simulation'],
        description='Simulation mode: viewer (just view model) or simulation (run full Simulation.py)'
    )
    
    # Create MuJoCo process using OpaqueFunction to handle LaunchConfiguration
    def create_mujoco_process(context):
        sim_mode = context.launch_configurations.get('sim_mode', 'viewer')
        
        if sim_mode == 'simulation':
            # Run the full simulation with VMC control
            # Python files are installed to share/wheel_leg_mujoco/
            sim_script = os.path.join(pkg_share, 'Simulation.py')
            return [ExecuteProcess(
                cmd=['python3', sim_script],
                output='screen',
                cwd=pkg_share  # Set working directory so imports work
            )]
        else:
            # Just view the model using MuJoCo viewer
            cmd_str = context.launch_configurations.get('mujoco_cmd', 'python3 -m mujoco.viewer')
            full_cmd = f'{cmd_str} --mjcf {mjcf_file}'
            return [ExecuteProcess(
                cmd=['bash', '-c', full_cmd],
                output='screen',
                cwd=mjcf_dir  # Set working directory to MJCF folder so mesh paths work
            )]
    
    mujoco_process = OpaqueFunction(function=create_mujoco_process)
    
    return LaunchDescription([
        mujoco_cmd_arg,
        sim_mode_arg,
        mujoco_process
    ])

