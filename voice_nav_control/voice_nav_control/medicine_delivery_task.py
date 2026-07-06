from .voice_nav_control import run


def main(args=None) -> None:
    run(args=args, node_name='medicine_delivery_task')
