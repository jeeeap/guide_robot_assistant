import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameter('enable_console_output', True)
        self.declare_parameter('enable_espeak', True)
        self.declare_parameter('espeak_voice', 'zh')          # 中文语音
        self.declare_parameter('espeak_speed', 145)            # 语速（词/分钟）
        self.declare_parameter('espeak_amplitude', 100)        # 音量 0-200
        self.declare_parameter('espeak_pitch', 50)             # 音调 0-99

        self.enable_console_output = bool(self.get_parameter('enable_console_output').value)
        self.enable_espeak = bool(self.get_parameter('enable_espeak').value)
        self._espeak_available = self._check_espeak() if self.enable_espeak else False

        self._speak_lock = threading.Lock()  # 防止多条TTS重叠播报

        self.status_publisher = self.create_publisher(String, '/tts_status', 10)
        self.subscription = self.create_subscription(String, '/tts_text', self.handle_tts_text, 10)

        mode_parts = []
        if self.enable_espeak and self._espeak_available:
            mode_parts.append('espeak-ng语音合成')
        if self.enable_console_output:
            mode_parts.append('控制台输出')
        mode = ' + '.join(mode_parts) if mode_parts else '静默模式'
        self.get_logger().info(f'TTS节点已启动 [{mode}]，订阅 /tts_text。')

    def _check_espeak(self) -> bool:
        try:
            result = subprocess.run(
                ['espeak-ng', '--version'],
                capture_output=True, timeout=3
            )
            if result.returncode == 0:
                self.get_logger().info('espeak-ng 可用，将进行真实语音播报。')
                return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
        self.get_logger().warning(
            'espeak-ng 不可用，仅控制台输出。\n'
            '安装命令: sudo apt install espeak-ng espeak-ng-data-zh'
        )
        return False

    def handle_tts_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        if self.enable_console_output:
            self.get_logger().info(f'[机器人播报] {text}')

        self._publish_status('speaking')

        if self.enable_espeak and self._espeak_available:
            # 在后台线程播报，避免阻塞 ROS2 spin
            thread = threading.Thread(target=self._speak_blocking, args=(text,), daemon=True)
            thread.start()
        else:
            self._publish_status('finished')

    def _speak_blocking(self, text: str):
        voice = self.get_parameter('espeak_voice').value
        speed = int(self.get_parameter('espeak_speed').value)
        amplitude = int(self.get_parameter('espeak_amplitude').value)
        pitch = int(self.get_parameter('espeak_pitch').value)

        with self._speak_lock:
            try:
                subprocess.run(
                    ['espeak-ng',
                     '-v', voice,
                     '-s', str(speed),
                     '-a', str(amplitude),
                     '-p', str(pitch),
                     text],
                    timeout=20,
                    capture_output=True,
                )
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f'espeak-ng 播报超时: {text}')
            except Exception as exc:
                self.get_logger().warning(f'espeak-ng 播报失败: {exc}')
            finally:
                self._publish_status('finished')

    def _publish_status(self, state: str):
        msg = String()
        msg.data = state
        self.status_publisher.publish(msg)


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
