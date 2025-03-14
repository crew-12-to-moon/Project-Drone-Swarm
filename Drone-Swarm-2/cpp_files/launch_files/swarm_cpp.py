from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ardupilot_takeoff',
            executable='leader',
            output='screen'
        ),
        Node(
            package='ardupilot_takeoff',
            executable='follower',
            output='screen'
        )
    ])
