#!/usr/bin/env python3
"""
MuJoCo with ROS2 Control Launch File
启动MuJoCo仿真环境并集成ros2_control力矩控制器
"""

import os
import xacro
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('cod_2026_balance')
    bringup_share = get_package_share_directory('bringup')
    
    # 自动检测 MuJoCo 库路径并设置 LD_LIBRARY_PATH
    mujoco_lib_path = None
    try:
        mujoco_python_path = subprocess.run(
            ['python3', '-c', 'import mujoco; import os; print(os.path.dirname(mujoco.__file__))'],
            capture_output=True,
            text=True,
            check=True
        ).stdout.strip()
        if os.path.exists(mujoco_python_path):
            # 检查是否存在任何版本的 libmujoco.so 文件
            import glob
            mujoco_lib_files = glob.glob(os.path.join(mujoco_python_path, 'libmujoco.so*'))
            if mujoco_lib_files:
                mujoco_lib_path = mujoco_python_path
    except Exception as e:
        print(f"Warning: Could not auto-detect MuJoCo library path: {e}")
        pass
    
    # 设置 LD_LIBRARY_PATH 环境变量
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    if mujoco_lib_path:
        if ld_library_path:
            ld_library_path = f"{mujoco_lib_path}:{ld_library_path}"
        else:
            ld_library_path = mujoco_lib_path
    
    # MJCF文件路径
    mjcf_file = os.path.join(pkg_share, 'MJCF', 'COD-2026RoboMaster-Balance.xml')
    
    # 声明启动参数
    use_mujoco_arg = DeclareLaunchArgument(
        'use_mujoco',
        default_value='true',
        description='Whether to use MuJoCo hardware interface'
    )
    
    # 声明启动参数：是否使用 mujoco.viewer 作为交互式可视化窗口
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='Whether to launch mujoco.viewer for interactive visualization (supports joint dragging)'
    )
    
    # 获取URDF（设置use_mujoco参数为true）
    xacro_file = os.path.join(pkg_share, 'urdf', 'cod_balance_robot.xacro')
    
    # 使用xacro处理文件生成URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_mujoco': 'true'})
    robot_description = {'robot_description': doc.toxml()}
    
    # 控制器配置文件路径
    controller_config_file = os.path.join(bringup_share, 'config', 'cod_balance_controllers.yaml')

    # MuJoCo ROS2 Control节点 - 该节点同时包含ROS2 Control和MuJoCo仿真器
    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        name='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': mjcf_file}
        ]
    )

    # Robot State Publisher节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 静态TF发布器 - base_link到base_footprint
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    )

    # 加载Joint State Broadcaster控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 加载Joint Group Effort Controller控制器
    load_joint_group_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_effort_controller'],
        output='screen'
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

    # 设置环境变量
    env_actions = []
    if ld_library_path:
        env_actions.append(SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path))
    
    # 创建 MuJoCo viewer 进程（用于交互式可视化，支持关节拖动）
    def create_mujoco_viewer(context):
        use_viewer = LaunchConfiguration('use_viewer').perform(context) == 'true'
        if use_viewer:
            full_cmd = f'python3 -m mujoco.viewer --mjcf {mjcf_file}'
            return [ExecuteProcess(
                cmd=['bash', '-c', full_cmd],
                output='screen',
                cwd=os.path.dirname(mjcf_file)  # Set working directory to MJCF folder so mesh paths work
            )]
        return []
    
    mujoco_viewer_process = OpaqueFunction(function=create_mujoco_viewer)
    
    return LaunchDescription([
        use_mujoco_arg,
        use_viewer_arg,
        
        # 设置环境变量
        *env_actions,
        
        # 启动MuJoCo viewer（交互式可视化窗口，支持关节拖动）
        mujoco_viewer_process,
        
        # 启动MuJoCo ROS2 Control节点（包含仿真器和控制器管理器）
        node_mujoco_ros2_control,
        
        # 发布机器人状态
        node_robot_state_publisher,
        static_tf_node,
        
        # 使用事件处理器按顺序加载控制器
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_group_effort_controller],
            )
        ),
        
        # 控制转换节点
        control_converter_node,
        
        # 关节力矩控制器节点
        joint_torque_controller_node,
    ])


# 使用说明:
# 1. 确保已编译 mujoco_ros2_control 包（位于 src/hardware/mujoco_ros2_control）
#
# 2. 启动仿真（默认会启动交互式 mujoco.viewer 窗口）:
#    ros2 launch bringup mujoco_ros2_control.launch.py
#
# 3. 如果不想启动 mujoco.viewer 窗口（只使用 mujoco_ros2_control 的内置窗口）:
#    ros2 launch bringup mujoco_ros2_control.launch.py use_viewer:=false
#
# 4. 在 mujoco.viewer 窗口中:
#    - 左键拖动：旋转视角
#    - 右键拖动：移动视角
#    - 中键拖动/滚轮：缩放
#    - 按住 Ctrl 并左键点击关节：选中关节
#    - 选中关节后拖动：拖动关节
#
# 5. 发布控制命令到 /control_input 话题来控制机器人
#
# 6. 查看关节状态:
#    ros2 topic echo /joint_states
#
# 注意: 
# - mujoco_ros2_control 节点同时包含了 ROS2 Control 节点和 MuJoCo 仿真器
# - mujoco.viewer 提供完整的交互功能（包括关节拖动），而 mujoco_ros2_control 的内置窗口只支持相机控制
# - 两个窗口会显示同一个模型，但 mujoco.viewer 更适合交互式操作

