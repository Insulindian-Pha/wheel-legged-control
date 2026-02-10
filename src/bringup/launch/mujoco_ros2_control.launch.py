#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

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
    
    # Get MJCF model file path
    mujoco_model_path = PathJoinSubstitution([cod_balance_pkg_share, "MJCF", "COD-2026RoboMaster-Balance.xml"])

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

    robot_description_str = robot_description_content.perform(context)
    
    # Replace package:// paths with absolute paths for mesh files
    cod_balance_pkg_share_path = cod_balance_pkg_share.perform(context)
    robot_description_str = robot_description_str.replace(
        'package://cod_2026_balance/', 
        'file://' + cod_balance_pkg_share_path + '/'
    )

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

    parameters_file = PathJoinSubstitution([FindPackageShare("bringup"), "config", "cod_balance_controllers.yaml"])

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
    controllers_to_spawn = ["joint_state_broadcaster", "imu_broadcaster", "vmc_controller"]
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
