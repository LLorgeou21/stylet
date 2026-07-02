#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Chemins ---
    world_path = os.path.join(
        get_package_share_directory("stylet_simulation"), "worlds", "stylet.world"
    )
    xacro_file = os.path.join(
        get_package_share_directory("stylet_description"), "urdf", "robot.urdf.xacro"
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    # --- Bloc 1 : où Gazebo doit chercher model://target ---
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(get_package_share_directory("stylet_simulation"), "models")
        + ":"
        + os.path.join(get_package_share_directory("stylet_simulation"), "worlds")
        + ":"
        + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    # --- Bloc 2 : démarrer Gazebo avec notre monde ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-v 4 " + world_path}.items(),
    )

    # --- Bloc 3 : publier robot_description (comme en 1.3/1.4) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # --- Bloc 4 : faire apparaître le robot DANS Gazebo ---
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-name", "ur5e"],
    )

    # --- Bloc 5 : pont Gazebo -> ROS 2 pour le capteur de force/couple ---
    gz_ft_topic = "/world/stylet_world/model/ur5e/joint/needle_joint/sensor/needle_ft_sensor/forcetorque"
    ft_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            gz_ft_topic + "@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench",
        ],
        remappings=[
            (gz_ft_topic, "/stylet/haptics/wrench"),
        ],
    )

    # --- Assemblage final : TOUT doit être dans cette liste ---
    return LaunchDescription(
        [
            set_resource_path,
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            ft_bridge,
        ]
    )
