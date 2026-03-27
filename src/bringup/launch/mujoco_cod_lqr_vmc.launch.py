#!/usr/bin/env python3
#
# Launch file for MuJoCo simulation with VMC and LQR controllers
# Starts both VMC controller and LQR controller for wheel-legged robot balance control
#

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    cod_balance_pkg_share = FindPackageShare("cod_2026_balance")
    
    # Get MJCF model file path (use env.xml which includes robot.xml)
    mujoco_model_path = PathJoinSubstitution([cod_balance_pkg_share, "MJCF", "env.xml"])

    # Build robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([cod_balance_pkg_share, "urdf", "cod_balance_robot.xacro"]),
            " use_mujoco:=true",
            " mujoco_model:=",
            mujoco_model_path,
        ]
    )

    # Keep package:// mesh URIs so Foxglove can request assets through foxglove_bridge.
    robot_description_str = robot_description_content.perform(context)

    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    nodes = []

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    parameters_file = PathJoinSubstitution([FindPackageShare("bringup"), "config", "cod_lqr_vmc_config.yaml"])

    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(parameters_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            on_exit=Shutdown(),
        )
    )

    # Add controller spawners
    # Start VMC controller first, then LQR controller (LQR depends on VMC state topic)
    controllers_to_spawn = ["joint_state_broadcaster", "imu_broadcaster", "vmc_controller", "lqr_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", parameters_file],
                output="both",
            )
        )

    return nodes


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
