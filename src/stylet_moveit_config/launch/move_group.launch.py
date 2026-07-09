from launch_ros.actions import Node
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # use_sim_time=True is required with Gazebo (gz_ros2_control, ADR-033):
    # without it, move_group compares /joint_states' sim-time timestamp against
    # the wall clock and rejects the robot's current state as "stale" ("Check
    # clock synchronization") - the root cause of both planning AND execution
    # failures observed while testing the approach phase.
    moveit_config = MoveItConfigsBuilder("stylet", package_name="stylet_moveit_config").to_dict()
    moveit_config["use_sim_time"] = True

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config],
    )

    return LaunchDescription([move_group_node])
