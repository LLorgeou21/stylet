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

    gz_lidar1_topic = "/lidar_1/points"
    gz_lidar2_topic = "/lidar_2/points"
    gz_lidar3_topic = "/lidar_3/points"
    gz_lidar4_topic = "/lidar_4/points"
    gz_lidar5_topic = "/lidar_5/points"
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            gz_lidar1_topic + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            gz_lidar2_topic + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            gz_lidar3_topic + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            gz_lidar4_topic + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            gz_lidar5_topic + "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
    )

    # --- Bloc 6 : TF statique pour les 3 LiDAR (fixes, définis en SDF pur, jamais publiés en TF sinon) ---
    tf_lidar_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0.40", "--z", "0.02",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "lidar_rig/lidar_front/lidar_front_sensor",
        ],
    )
    tf_lidar_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.075", "--y", "0.075", "--z", "0.02",
            "--roll", "0", "--pitch", "0", "--yaw", "0.785",
            "--frame-id", "world",
            "--child-frame-id", "lidar_rig/lidar_left/lidar_left_sensor",
        ],
    )
    tf_lidar_top = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.30", "--y", "0.30", "--z", "0.40",
            "--roll", "0", "--pitch", "1.571", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "lidar_rig/lidar_top/lidar_top_sensor",
        ],
    )
    tf_lidar_back = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.45", "--y", "0.0", "--z", "0.02",
            "--roll", "0", "--pitch", "0", "--yaw", "2.111",
            "--frame-id", "world",
            "--child-frame-id", "lidar_rig/lidar_back/lidar_back_sensor",
        ],
    )
    tf_lidar_side = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.25", "--y", "0.60", "--z", "0.10",
            "--roll", "0", "--pitch", "0", "--yaw", "-1.5708",
            "--frame-id", "world",
            "--child-frame-id", "lidar_rig/lidar_side/lidar_side_sensor",
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
            lidar_bridge,
            tf_lidar_front,
            tf_lidar_left,
            tf_lidar_top,
            tf_lidar_back,
            tf_lidar_side,
        ]
    )
