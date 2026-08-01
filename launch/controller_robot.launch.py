from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'bayesian_shared_control'
    default_params = os.path.join(
        get_package_share_directory(pkg_name), 'config/sc_params_robot.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to ROS 2 parameters file.'
    )

    params_file = LaunchConfiguration('params_file')

    ns_group = GroupAction([
        PushRosNamespace('bayesian_shared_control'),

        Node(
            package=pkg_name,
            executable='shared_control_node',
            name='bayesian_shared_control',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
            remappings=[
                ('/shared_control_teleop/cmd_vel',
                 '/robot/shared_control_teleop/cmd_vel'),
            ]
        ),
    ])

    return LaunchDescription([
        params_file_arg,
        ns_group
    ])
