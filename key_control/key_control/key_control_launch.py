import sys

from key_control.key_control import run_key_control


def main():
    try:
        with open('/dev/tty', 'r', encoding='utf-8', buffering=1) as input_stream:
            return run_key_control(input_stream)
    except OSError:
        print('key_control launch mode requires access to /dev/tty.', flush=True)
        print('Run `ros2 launch key_control key_control.launch.py` from a local terminal session.', flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
