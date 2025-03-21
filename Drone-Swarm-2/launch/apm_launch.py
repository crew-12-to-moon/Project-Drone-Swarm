from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'fcu_url': 'udp://127.0.0.1:14550@14555',
        'gcs_url': '',
        'target_system_id': 1,
        'target_component_id': 1,
        'fcu_protocol': 'v2.0',
        'namespace': 'mavros/uas_1_1'
    }

    remaps = [
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
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[params],
            remappings=remaps,
            namespace='mavros/uas_1_1'
        )
    ])
