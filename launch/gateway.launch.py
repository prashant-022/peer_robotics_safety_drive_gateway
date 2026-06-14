from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="drive_safety_gateway",
                executable="drive_safety_gateway",
                name="drive_safety_gateway",
                output="screen",
                parameters=[
                    {
                        "slow_distance": 1.0,
                        "stop_distance": 0.3,
                        "timeout_sec": 0.5,
                        "publish_rate": 20.0,
                        "max_linear_vel": 1.0,
                        "max_angular_vel": 1.5,
                    }
                ],
            )
        ]
    )
