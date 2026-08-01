from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # change package name to yours
    pkg_name = "bayesian_shared_control"

    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        "config",
        "teleop_joy.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[params_file],
        remappings=[("cmd_vel", "/user_cmd")]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
