from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # procedure_planner builds its own MoveGroupInterface (3.5) - it needs the
    # same MoveIt parameters (kinematics.yaml, SRDF...) as move_group/moveit_rviz,
    # otherwise "No kinematics plugins defined" (the IK solver internal to
    # MoveGroupInterface, not /compute_ik, which talks to the real move_group
    # and already works fine).
    moveit_config = MoveItConfigsBuilder("stylet", package_name="stylet_moveit_config").to_dict()

    procedure_planner = Node(
        package="stylet_planning",
        executable="procedure_planner",
        output="screen",
        parameters=[moveit_config],
    )

    return LaunchDescription([procedure_planner])
