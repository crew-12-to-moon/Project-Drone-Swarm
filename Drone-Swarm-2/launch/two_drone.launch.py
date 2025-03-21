import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    world_path = os.path.join(get_package_share_directory("drone_swarm"), "worlds", "two_drone.world")
    #ardupilot_gazebo_path = os.path.join(get_package_share_directory("ardupilot_gazebo"))
    ardupilot_gazebo_path = "/home/ojas/gz_ws/src/ardupilot_gazebo"

    param1 = {
        'fcu_url': 'udp://127.0.0.1:14550@14555',
        'gcs_url': '',
        'target_system_id': 1,
        'target_component_id': 1,
        'fcu_protocol': 'v2.0',
        'namespace': 'mavros/uas_1_1'
    }

    remap1 = [
        # Isolate the navigation goal topic
        ('/move_base_simple/goal', 'move_base_simple/goal'),
        # Isolate tf topics
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        # Isolate the MAVLink sink/source topics
        ('/uas1/mavlink_sink', 'uas1/mavlink_sink'),
        ('/uas1/mavlink_source', 'uas1/mavlink_source')
    ]

    return LaunchDescription([
        # Launch Gazebo with the multi-UAV world
        ExecuteProcess(
            cmd=["gz", "sim", "-v4", "-r", world_path],
            output="screen"
        ),

        # Start MAVROS for Drone 1
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[param1],
            remappings=remap1,
            namespace='mavros/uas_1_1'
        ),

        # Start MAVROS for Drone 2
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://127.0.0.1:14560@14565',
                'gcs_url': '',
                'target_system_id': 2,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'namespace': 'mavros/uas_1_2'
            }],
            remappings=[
                # If you want to isolate the navigation goal topic
                ('/move_base_simple/goal', 'move_base_simple/goal'),
                # If you want to isolate tf
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                # If you want to isolate the MAVLink sink/source
                ('/uas2/mavlink_sink', 'uas2/mavlink_sink'),
                ('/uas2/mavlink_source', 'uas2/mavlink_source'),
                # Typically do NOT rename /rosout or /parameter_events
            ],
            namespace='mavros/uas_1_2'
        ),
    ])

