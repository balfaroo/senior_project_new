from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bryan_offboard',
            executable='offboard_complete',
            name='offboard_complete'
        ),
        Node(
            package='bryan_offboard',
            executable='depth_cam_node',
            name='depth_cam_node'
        ),
        Node(
            package='bryan_offboard',
            executable='bottom_cam_node',
            name='bottom_cam_node'
        )
    ])