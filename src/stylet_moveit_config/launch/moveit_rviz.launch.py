import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # generate_moveit_rviz_launch() (moveit_configs_utils) does NOT pass
    # robot_description/robot_description_semantic to the rviz2 node - only
    # planning_pipelines/robot_description_kinematics/joint_limits (verified in
    # the installed launches.py). Observed consequence: empty RobotModel
    # ("Parameter not set" for robot_description on /rviz) and PlanningScene
    # never loaded. Same fix as move_group.launch.py: pass the FULL dict
    # (.to_dict()) rather than the auto-generated function.
    moveit_config = MoveItConfigsBuilder("stylet", package_name="stylet_moveit_config").to_dict()
    moveit_config["use_sim_time"] = True

    rviz_config = os.path.join(
        MoveItConfigsBuilder("stylet", package_name="stylet_moveit_config").to_moveit_configs().package_path,
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[moveit_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        rviz_node,
    ])
