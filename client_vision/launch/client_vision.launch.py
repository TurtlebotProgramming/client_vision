from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='client_vision',
            executable='vision_subscriber',
            name='client_vision',
            output='screen',
        ),
        Node(
            package='client_vision',
            executable='detection_refiner',
            name='detection_refiner',
            output='screen',
        ),
    ])
