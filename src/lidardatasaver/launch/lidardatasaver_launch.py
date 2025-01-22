from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidardatasaver',
            executable='lidardatasaver_node',
            name='lidardatasaver',
            output='screen'
        )
    ])
