from setuptools import setup
from glob import glob

package_name = 'robot_ai'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'dashscope',
        'numpy',
        'sounddevice',
        'requests',
        'beautifulsoup4',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='user@todo.todo',
    description='Robot AI voice assistant with ASR, LLM, TTS, memory, web tools, and workspace file reading.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_ai = robot_ai.voice_chat_node:main',
            'robot_ai_hello_test = robot_ai.hello_text_test:main',
            'robot_ai_mic_test = robot_ai.mic_test:main',
        ],
    },
)
