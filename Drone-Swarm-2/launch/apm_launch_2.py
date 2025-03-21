from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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

