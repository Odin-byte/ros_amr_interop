import rclpy
from rclpy.node import Node


class LukaAdapter(Node):
    def __init__(self) -> None:
        super().__init__('luka_adapter')

        self.get_logger().info(f"LUKA adapter started")


def main():
    rclpy.init()
    print('Starting luka_adapter')

    adapter = LukaAdapter()

    try:
        rclpy.spin(adapter)
    except KeyboardInterrupt:
        pass
    except BaseException:
        raise
    finally:
        adapter.on_shutdown()
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
