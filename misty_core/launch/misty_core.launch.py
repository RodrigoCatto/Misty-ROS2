from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='misty_core',
            executable='misty_low_level',
            name='misty_low_level_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'ip_address': '192.168.1.3'}
            ]
        )
    ])