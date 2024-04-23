from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bryan_offboard',
            executable='offboard_alternative_bottom_facing_rgb_lateral',
            name='offboard_new'
        ),
        Node(
            package='bryan_offboard',
            executable='depth_cam_later_rgb',
            name='new_depth'
        ),
        Node(
            package='bryan_offboard',
            executable='bottom_facing_cam',
            name='new_bottom'
        )
    ])