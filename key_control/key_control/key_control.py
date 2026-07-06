import os
import select
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


HELP_MSG = """
Competition keyboard control
----------------------------
Moving:
        i
   j    k    l
        ,

i/, : forward/backward
j/l : left/right turn
k or space : stop immediately

q/z : increase/decrease all speeds by 10%
w/x : increase/decrease linear speed by 10%
e/c : increase/decrease angular speed by 10%

CTRL-C to quit
"""


MOVE_BINDINGS = {
    'i': (1.0, 0.0),
    ',': (-1.0, 0.0),
    'j': (0.0, 1.0),
    'l': (0.0, -1.0),
}


SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(input_stream, original_settings):
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8')
        return ''

    tty.setraw(input_stream.fileno())
    readable, _, _ = select.select([input_stream], [], [], 0.1)
    key = input_stream.read(1) if readable else ''
    termios.tcsetattr(input_stream, termios.TCSADRAIN, original_settings)
    return key


def print_vels(linear_speed, angular_speed):
    print(f'currently: linear {linear_speed:.3f} m/s angular {angular_speed:.3f} rad/s', flush=True)


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def make_twist(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    return twist


def run_key_control(input_stream):
    original_settings = None
    if os.name != 'nt':
        if not input_stream.isatty():
            print('key_control requires an interactive terminal (TTY).', flush=True)
            print('Start it with `ros2 run key_control key_control` from a local terminal session.', flush=True)
            return 1
        original_settings = termios.tcgetattr(input_stream)

    rclpy.init()
    node = rclpy.create_node('key_control')
    node.declare_parameter('cmd_vel_topic', 'cmd_vel')
    node.declare_parameter('linear_speed', 0.20)
    node.declare_parameter('angular_speed', 0.80)
    node.declare_parameter('max_linear_speed', 0.50)
    node.declare_parameter('max_angular_speed', 1.50)
    node.declare_parameter('linear_step', 0.04)
    node.declare_parameter('angular_step', 0.15)

    cmd_vel_topic = node.get_parameter('cmd_vel_topic').get_parameter_value().string_value
    publisher = node.create_publisher(Twist, cmd_vel_topic, QoSProfile(depth=10))

    linear_speed = node.get_parameter('linear_speed').get_parameter_value().double_value
    angular_speed = node.get_parameter('angular_speed').get_parameter_value().double_value
    max_linear_speed = node.get_parameter('max_linear_speed').get_parameter_value().double_value
    max_angular_speed = node.get_parameter('max_angular_speed').get_parameter_value().double_value
    target_linear = 0.0
    target_angular = 0.0
    control_linear = 0.0
    control_angular = 0.0
    idle_count = 0

    linear_step = node.get_parameter('linear_step').get_parameter_value().double_value
    angular_step = node.get_parameter('angular_step').get_parameter_value().double_value

    linear_speed = clamp(linear_speed, 0.0, max_linear_speed)
    angular_speed = clamp(angular_speed, 0.0, max_angular_speed)

    print(HELP_MSG, flush=True)
    print(f'publishing to: {cmd_vel_topic}', flush=True)
    print(f'max limits: linear {max_linear_speed:.3f} m/s angular {max_angular_speed:.3f} rad/s', flush=True)
    node.get_logger().info('keyboard control is ready; focus this terminal and press i/j/k/l/,')
    print_vels(linear_speed, angular_speed)

    try:
        while True:
            key = get_key(input_stream, original_settings)

            if key in MOVE_BINDINGS:
                direction_linear, direction_angular = MOVE_BINDINGS[key]
                target_linear = linear_speed * direction_linear
                target_angular = angular_speed * direction_angular
                idle_count = 0
            elif key in SPEED_BINDINGS:
                linear_speed = clamp(
                    linear_speed * SPEED_BINDINGS[key][0],
                    0.0,
                    max_linear_speed,
                )
                angular_speed = clamp(
                    angular_speed * SPEED_BINDINGS[key][1],
                    0.0,
                    max_angular_speed,
                )
                print_vels(linear_speed, angular_speed)
                idle_count = 0
            elif key in (' ', 'k'):
                target_linear = 0.0
                target_angular = 0.0
                control_linear = 0.0
                control_angular = 0.0
                idle_count = 0
            else:
                idle_count += 1
                if idle_count > 4:
                    target_linear = 0.0
                    target_angular = 0.0

            if key == '\x03':
                break

            if target_linear > control_linear:
                control_linear = min(target_linear, control_linear + linear_step)
            elif target_linear < control_linear:
                control_linear = max(target_linear, control_linear - linear_step)

            if target_angular > control_angular:
                control_angular = min(target_angular, control_angular + angular_step)
            elif target_angular < control_angular:
                control_angular = max(target_angular, control_angular - angular_step)

            publisher.publish(make_twist(control_linear, control_angular))

    finally:
        publisher.publish(make_twist(0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()
        if os.name != 'nt':
            termios.tcsetattr(input_stream, termios.TCSADRAIN, original_settings)


def main():
    return run_key_control(sys.stdin)


if __name__ == '__main__':
    main()
