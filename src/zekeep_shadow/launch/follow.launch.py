from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="zekeep_shadow", executable="shadow_follower", prefix="python3", output="screen"),
        Node(package="zekeep_shadow", executable="shadow_vision", prefix="python3", output="screen"),
    ])
