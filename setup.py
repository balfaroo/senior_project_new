from setuptools import find_packages, setup

package_name = 'bryan_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'offboard_track_april = bryan_offboard.offboard_track_april:main'
        ],
    },
)
