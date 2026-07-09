#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Paths ---
    world_path = os.path.join(
        get_package_share_directory("stylet_simulation"), "worlds", "stylet.world"
    )
    # Combined xacro (stylet_moveit_config): robot.urdf.xacro (stylet_description)
    # + the <ros2_control> tags (gz_ros2_control/GazeboSimSystem plugin, ADR-033) -
    # a SINGLE /robot_description shared between Gazebo and MoveIt, rather than
    # two different xacros published by two competing robot_state_publisher
    # instances (see ADR-030, the conflict this had with demo.launch.py).
    xacro_file = os.path.join(
        get_package_share_directory("stylet_moveit_config"), "config", "stylet.urdf.xacro"
    )
    initial_positions_file = os.path.join(
        get_package_share_directory("stylet_moveit_config"), "config", "initial_positions.yaml"
    )
    robot_description = xacro.process_file(
        xacro_file, mappings={"initial_positions_file": initial_positions_file}
    ).toxml()

    # --- Block 1: where Gazebo should look for model://target ---
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(get_package_share_directory("stylet_simulation"), "models")
        + ":"
        + os.path.join(get_package_share_directory("stylet_simulation"), "worlds")
        + ":"
        + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    # --- Block 2: start Gazebo with our world ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        # -r: starts the simulation running (not paused). Without it, time
        # never advances, gz_ros2_control never gets "ticked" by Gazebo, and
        # controller activation blocks forever ("Switch controller timed out").
        launch_arguments={"gz_args": "-v 4 -r " + world_path}.items(),
    )

    # --- Block 3: publish robot_description ---
    # use_sim_time=True: now that there's a /clock bridge (block 4ter), every
    # time-dependent node (TF here) must use simulation time, not the wall
    # clock - otherwise the same clock-mismatch issue as the RViz Tool hits.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # --- Block 4: spawn the robot INSIDE Gazebo ---
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-name", "ur5e"],
    )

    # --- Block 4bis: start the ros2_control controllers (hosted by Gazebo via
    # gz_ros2_control, ADR-033) - the spawner already waits on its own for
    # /controller_manager to be ready, no need to wait explicitly here.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )
    # Insertion actuator (Phase 3.6) - separate controller, see
    # ros2_controllers.yaml/robot.urdf.xacro for why.
    needle_insertion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["needle_insertion_controller"],
    )

    # --- Block 4ter: simulation clock -> ROS 2 bridge (needed for
    # gz_ros2_control, ADR-033 - without /clock, the controller_manager hosted
    # by Gazebo has no coherent notion of time). ---
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # --- Block 5: Gazebo -> ROS 2 bridge for the force/torque sensor ---
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

    # --- Block 6: static TF for the 5 LiDAR sensors (fixed, defined in pure SDF, never published as TF otherwise) ---
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

    # --- Final assembly: EVERYTHING must be in this list ---
    return LaunchDescription(
        [
            set_resource_path,
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            needle_insertion_controller_spawner,
            clock_bridge,
            ft_bridge,
            lidar_bridge,
            tf_lidar_front,
            tf_lidar_left,
            tf_lidar_top,
            tf_lidar_back,
            tf_lidar_side,
        ]
    )
