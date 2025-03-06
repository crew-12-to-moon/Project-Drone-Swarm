import launch
import launch_ros.actions
from launch.actions import GroupAction, ExecuteProcess

REMOTE_PI_USER = "pi"  # Change to your Raspberry Pi's username
REMOTE_PI_IP = "192.168.1.2"  # Change to your second Pi's IP

def generate_launch_description():
    return launch.LaunchDescription([
        # Run nodes locally on Pi 1
        GroupAction([
            launch_ros.actions.Node(
                package='package1',
                executable='node1',
                name='drone1_controller',
            ),
            launch_ros.actions.Node(
                package='package2',
                executable='node2',
                name='drone1_sensors',
            ),
        ]),

        # Run nodes remotely on Pi 2 via SSH
        ExecuteProcess(
            cmd=[
                "ssh", f"{REMOTE_PI_USER}@{REMOTE_PI_IP}",
                "ros2 launch package3 drone2_launch.py"
            ],
            output="screen"
        ),
    ])
