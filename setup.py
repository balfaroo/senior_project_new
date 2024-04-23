from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bryan_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryan',
    maintainer_email='bryan@todo.todo',
    description='TODO: Package description',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_track_april = bryan_offboard.offboard_track_april:main',
            'bottom_april = bryan_offboard.bottom_april:main',
            'bottom_cam_node = bryan_offboard.bottom_cam_node:main',
            'bottom_cam_node_listener = bryan_offboard.bottom_cam_node_listener:main',
            'depth_cam_node = bryan_offboard.depth_cam_node:main',
            'depth_cam_listener = bryan_offboard.depth_cam_listener:main',
            'offboard_complete = bryan_offboard.offboard_complete:main',
            'new_bottom = bryan_offboard.bottom_facing_cam:main',
            'new_depth = bryan_offboard.depth_cam_lateral_rgb:main',
            'new_depth_listener = bryan_offboard.depth_listener_new:main',
            'offboard_new = bryan_offboard.offboard_alternative_bottom_facing_rgb_lateral:main'
        ],
    },
)
