#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    cod_balance_pkg_share = FindPackageShare("cod_2026_balance")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution([cod_balance_pkg_share, "urdf", "cod_balance_robot.xacro"]),
                ]
            ),
            value_type=str,
        )
    }

    parameters_file = LaunchConfiguration("controllers_file")

    controller_manager_parameters = [
        robot_description,
        ParameterFile(parameters_file, allow_substs=True),
        {"use_sim_time": False},
    ]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": False}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=controller_manager_parameters,
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
            on_exit=Shutdown(),
        ),
    ]

    controllers_to_spawn = [
        "joint_state_broadcaster",
        "vmc_controller",
        "lqr_controller",
    ]
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
    default_controllers = PathJoinSubstitution(
        [FindPackageShare("bringup"), "config", "bm_balance.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=default_controllers,
                description="Controller parameter file for real hardware LQR+VMC bringup.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
