from setuptools import setup

package_name = 'key_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/key_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='Keyboard teleoperation package for competition use.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'key_control = key_control.key_control:main',
            'key_control_launch = key_control.key_control_launch:main',
        ],
    },
)
