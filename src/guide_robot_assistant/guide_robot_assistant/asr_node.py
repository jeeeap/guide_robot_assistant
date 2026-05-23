"""
ASR节点：语音识别 → /raw_text
识别链路（从上到下逐级兜底）：
  1. Google Speech Recognition（在线，zh-CN）
  2. 离线 Sphinx（若安装 PocketSphinx）
  3. 文本输入（终端键盘，最终兜底）
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AsrNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        self.declare_parameter('use_microphone', True)
        self.declare_parameter('language', 'zh-CN')
        self.declare_parameter('recognition_timeout_sec', 5.0)
        self.declare_parameter('phrase_time_limit_sec', 8.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('ambient_noise_duration', 1.0)

        self.raw_text_pub = self.create_publisher(String, '/raw_text', 10)
        self.status_pub = self.create_publisher(String, '/asr_status', 10)

        self._sr = None
        self._recognizer = None
        self._sr_available = self._init_sr()

        use_mic = bool(self.get_parameter('use_microphone').value)

        if self._sr_available and use_mic:
            thread = threading.Thread(target=self._mic_loop, daemon=True)
            thread.start()
            self.get_logger().info('ASR节点已启动：麦克风监听模式（Google → Sphinx → 文本兜底）。')
        else:
            if not self._sr_available:
                self.get_logger().warning(
                    'speech_recognition 未安装或麦克风不可用，使用文本输入兜底。\n'
                    '安装命令: pip install SpeechRecognition pyaudio'
                )
            thread = threading.Thread(target=self._text_fallback_loop, daemon=True)
            thread.start()
            self.get_logger().info('ASR节点：文本输入模式（终端键盘兜底）。')

    # ── 初始化 speech_recognition ─────────────────────────────────────────
    def _init_sr(self) -> bool:
        try:
            import speech_recognition as sr
            self._sr = sr
            self._recognizer = sr.Recognizer()
            self._recognizer.energy_threshold = 300
            self._recognizer.dynamic_energy_threshold = True
            self._recognizer.pause_threshold = 0.8
            return True
        except ImportError:
            return False

    # ── 麦克风识别主循环 ──────────────────────────────────────────────────
    def _mic_loop(self):
        mic = self._init_microphone()
        if mic is None:
            self.get_logger().warning('麦克风初始化失败，切换到文本输入兜底。')
            self._publish_status('mic_init_failed')
            self._text_fallback_loop()
            return

        self.get_logger().info('麦克风就绪，开始持续监听...')
        self._publish_status('listening')
        while rclpy.ok():
            text = self._recognize_once(mic)
            if text:
                self._publish_text(text, source='asr_google')

    def _init_microphone(self):
        try:
            mic = self._sr.Microphone()
            ambient_dur = float(self.get_parameter('ambient_noise_duration').value)
            self.get_logger().info(f'校准环境噪声 ({ambient_dur}s)，请保持安静...')
            with mic as source:
                self._recognizer.adjust_for_ambient_noise(source, duration=ambient_dur)
            self.get_logger().info('噪声校准完成。')
            return mic
        except Exception as exc:
            self.get_logger().warning(f'麦克风初始化异常: {exc}')
            return None

    def _recognize_once(self, mic) -> str:
        timeout = float(self.get_parameter('recognition_timeout_sec').value)
        phrase_limit = float(self.get_parameter('phrase_time_limit_sec').value)
        max_retries = int(self.get_parameter('max_retries').value)
        lang = self.get_parameter('language').value

        for attempt in range(max_retries + 1):
            try:
                with mic as source:
                    self._publish_status('listening')
                    audio = self._recognizer.listen(
                        source, timeout=timeout, phrase_time_limit=phrase_limit
                    )

                # ① Google 在线识别（主路）
                try:
                    text = self._recognizer.recognize_google(audio, language=lang)
                    self._publish_status('recognized', text=text, engine='google')
                    self.get_logger().info(f'[ASR-Google] {text}')
                    return text
                except self._sr.UnknownValueError:
                    self._publish_status('unclear', attempt=attempt + 1)
                    if attempt < max_retries:
                        self.get_logger().info(f'听不清楚（第{attempt+1}次），请再说一遍...')
                        msg = String()
                        msg.data = f'对不起，没有听清，请再说一遍。'
                        # 不用tts_pub避免循环，直接打印
                    continue
                except self._sr.RequestError as exc:
                    self.get_logger().warning(f'Google ASR不可用: {exc}，尝试离线Sphinx兜底...')
                    # ② Sphinx 离线识别（二级兜底）
                    try:
                        text = self._recognizer.recognize_sphinx(audio)
                        self._publish_status('recognized', text=text, engine='sphinx_fallback')
                        self.get_logger().info(f'[ASR-Sphinx兜底] {text}')
                        return text
                    except Exception as sphinx_exc:
                        self._publish_status('all_engines_failed', error=str(sphinx_exc))
                        self.get_logger().warning('所有语音识别引擎均失败。')
                        return ''

            except self._sr.WaitTimeoutError:
                self._publish_status('timeout')
                # 超时是正常情况，继续监听
                continue
            except Exception as exc:
                self._publish_status('error', error=str(exc))
                self.get_logger().error(f'ASR异常: {exc}')
                time.sleep(1.0)
                break

        return ''

    # ── 文本输入兜底（③ 最终兜底）──────────────────────────────────────
    def _text_fallback_loop(self):
        self._publish_status('text_fallback_mode')
        self.get_logger().info('文本输入兜底模式：请在终端输入指令，回车发送。')
        while rclpy.ok():
            try:
                text = input('[ASR兜底] 请输入导盲机器人指令 > ').strip()
                if text:
                    self._publish_text(text, source='text_fallback')
            except (EOFError, KeyboardInterrupt):
                break

    # ── 发布 ──────────────────────────────────────────────────────────────
    def _publish_text(self, text: str, source: str):
        msg = String()
        msg.data = text
        self.raw_text_pub.publish(msg)
        self.get_logger().info(f'[{source}] → /raw_text: {text}')

    def _publish_status(self, state: str, **kwargs):
        payload = {'state': state, 'timestamp': time.time()}
        payload.update(kwargs)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
