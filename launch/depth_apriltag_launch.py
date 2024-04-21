from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bryan_offboard',
            executable='offboard_with_depth',
            name='offboard_with_depth'
        ),
        Node(
            package='bryan_offboard',
            executable='depth_cam_node',
            name='depth_cam_node'
        )
    ])