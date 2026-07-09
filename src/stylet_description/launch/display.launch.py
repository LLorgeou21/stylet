#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("stylet_description")

    # Load XACRO and convert it to a URDF string
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config = os.path.join(pkg, "rviz", "display.rviz")

    return LaunchDescription(
        [
            # Publishes robot_description on /robot_description
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            # GUI with sliders to move the joints
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            # 3D viewer
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
