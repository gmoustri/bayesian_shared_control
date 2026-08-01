from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[
            {"address": "0.0.0.0"},
            {"port": 9092}
        ],
        output="screen"
    )

    web_video = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        parameters=[
            {"port": 8080},
            {"address": "0.0.0.0"}
        ],
        output="screen"
    )

    web_server = ExecuteProcess(
        cmd=["python3", "-m", "http.server", "8000", "--bind", "0.0.0.0"],
        output="screen"
    )

    bag_recorder = Node(
        package="bayesian_shared_control",
        executable="remote_recorder.py",
        name="remote_recorder",
        output="screen"
    )

    return LaunchDescription([
        rosbridge,
        web_video,
        web_server,
        bag_recorder
    ])
