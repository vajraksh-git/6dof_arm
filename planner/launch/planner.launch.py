import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt configuration from your existing config package
    moveit_config = (
        MoveItConfigsBuilder("left_arm", package_name="6dof_arm_moveit_config")
        .robot_description(mappings={"use_fake_hardware": "true"})
        .to_dict()
    )

    # Launch the C++ planner node with MoveIt parameters attached
    planner_node = Node(
        package="planner",
        executable="planner",
        output="screen",
        parameters=[moveit_config]
    )

    return LaunchDescription([planner_node])