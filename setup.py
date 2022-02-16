from setuptools import setup
import os 
from glob import glob 

package_name = 'ros_roar_streamer'

setup(
    name=package_name,
    version='0.0.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roar',
    maintainer_email='roar@todo.todo',
    description='Control, State, RGB and Depth Streaming from iPhone to ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_streamer = ros_roar_streamer.control_streamer:main',
            'state_streamer = ros_roar_streamer.state_streamer:main',
            'rgb_streamer = ros_roar_streamer.rgb_streamer:main',
            'depth_streamer = ros_roar_streamer.depth_streamer:main',
            'manual_drive_with_pygame = ros_roar_streamer.manual_drive_with_pygame:main'
        ],
    },
)
