import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameter('enable_console_output', True)
        self.enable_console_output = self.get_parameter('enable_console_output').value
        self.status_publisher = self.create_publisher(String, '/tts_status', 10)
        self.subscription = self.create_subscription(String, '/tts_text', self.handle_tts_text, 10)
        self.get_logger().info('TTS 占位节点已启动，订阅 /tts_text。')

    def handle_tts_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        if self.enable_console_output:
            self.get_logger().info(f'机器人播报: {text}')

        status = String()
        status.data = 'finished'
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
