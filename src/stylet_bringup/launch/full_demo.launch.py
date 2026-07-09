import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Phase 3.7: a single launch file for the complete Phase 1-3 demo
    # (Gazebo+robot+target, LiDAR->fusion->filtering->ICP perception, MoveIt,
    # RViz plugin) - previously each piece was launched separately by hand.
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("stylet_simulation"), "launch", "sim.launch.py"
            )
        )
    )

    # Perception: started at the same time as the simulation - each node
    # already has its own retry/try-catch for TF/topics that aren't ready yet
    # (lookupTransform can throw before the TF buffer receives the static
    # transforms, already handled cleanly in each node).
    point_cloud_merger = Node(
        package="stylet_perception",
        executable="point_cloud_merger",
        output="screen",
    )
    cloud_preprocessor = Node(
        package="stylet_perception",
        executable="cloud_preprocessor",
        output="screen",
    )
    surface_registration = Node(
        package="stylet_perception",
        executable="surface_registration",
        output="screen",
    )

    # MoveIt/RViz/procedure_planner: delayed by a few seconds - Gazebo needs
    # to have started in "running" mode (not paused, -r already in
    # sim.launch.py, ADR-033) and the ros2_control controllers need time to
    # activate before move_group looks for a valid robot state. Same order as
    # the one used manually throughout Phase 3 (sim.launch.py starts and
    # stabilizes first, then everything else).
    move_group_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("stylet_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    )
                )
            )
        ],
    )
    moveit_rviz_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("stylet_moveit_config"),
                        "launch",
                        "moveit_rviz.launch.py",
                    )
                )
            )
        ],
    )
    procedure_planner_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("stylet_planning"),
                        "launch",
                        "procedure_planner.launch.py",
                    )
                )
            )
        ],
    )

    return LaunchDescription([
        sim_launch,
        point_cloud_merger,
        cloud_preprocessor,
        surface_registration,
        move_group_launch,
        moveit_rviz_launch,
        procedure_planner_launch,
    ])
