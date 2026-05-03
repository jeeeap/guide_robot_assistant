import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TextCommandNode(Node):
    def __init__(self):
        super().__init__('text_command_node')
        self.publisher = self.create_publisher(String, '/raw_text', 10)
        self._running = True
        self._input_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._input_thread.start()
        self.get_logger().info('文本命令节点已启动。请输入自然语言指令，例如：去302、带我去门口、依次去302和327。')

    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                text = input('请输入导盲机器人指令 > ').strip()
            except (EOFError, KeyboardInterrupt):
                self._running = False
                break

            if not text:
                continue

            msg = String()
            msg.data = text
            self.publisher.publish(msg)
            self.get_logger().info(f'已发布文本指令: {text}')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TextCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
