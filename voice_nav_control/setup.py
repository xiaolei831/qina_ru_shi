from setuptools import setup
import os
from glob import glob

package_name = 'voice_nav_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='Voice-based motion control and fixed-point navigation for the robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voice_nav_control = voice_nav_control.voice_nav_control:main',
            'medicine_delivery_task = voice_nav_control.medicine_delivery_task:main',
            'goal_pose_bridge = voice_nav_control.goal_pose_bridge:main',
        ],
    },
)
