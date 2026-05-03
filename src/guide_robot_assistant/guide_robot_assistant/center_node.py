import json
import os
import re
import time
import urllib.error
import urllib.request
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CenterNode(Node):
    def __init__(self):
        super().__init__('center_node')
        self.declare_parameter('use_llm', False)
        self.declare_parameter('llm_api_url', 'https://open.bigmodel.cn/api/paas/v4/chat/completions')
        self.declare_parameter('llm_model', 'glm-4-flash')
        self.declare_parameter('llm_api_key_env', 'ZHIPUAI_API_KEY')
        self.declare_parameter('llm_timeout', 8.0)

        self.command_publisher = self.create_publisher(String, '/navigation_command', 10)
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        self.status_publisher = self.create_publisher(String, '/task_status', 10)
        self.subscription = self.create_subscription(String, '/raw_text', self.handle_text, 10)

        self.location_aliases: Dict[str, List[str]] = {
            'entrance': ['门口', '入口', '大门', 'entrance'],
            'room_302': ['302', '302房间', '三零二', 'room302', 'room_302'],
            'room_327': ['327', '327房间', '三二七', 'room327', 'room_327'],
            'service_desk': ['服务台', '前台', '咨询台', '桌子', 'desk', 'service_desk'],
            'living_room': ['客厅', '大厅', 'living_room'],
            'kitchen': ['厨房', 'kitchen'],
        }
        self.cancel_words = ['停止', '取消', '别去了', '暂停导航', '取消导航', 'stop', 'cancel']
        self.get_logger().info('中心调度节点已启动，等待 /raw_text 指令。')

    def handle_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        command, meta = self.parse_command(text)
        self.publish_json(self.status_publisher, {
            'stage': 'parsed',
            'raw_text': text,
            'command': command,
            'parser_source': meta['parser_source'],
            'llm_latency': meta['llm_latency'],
            'llm_success': meta['llm_success'],
            'llm_error': meta['llm_error'],
        })

        nav_msg = String()
        nav_msg.data = json.dumps(command, ensure_ascii=False)
        self.command_publisher.publish(nav_msg)

        reply = command.get('reply', '')
        if reply:
            tts_msg = String()
            tts_msg.data = reply
            self.tts_publisher.publish(tts_msg)

        self.get_logger().info(f'解析结果: {nav_msg.data}')

    def parse_command(self, text: str) -> Tuple[Dict, Dict]:
        meta = {
            'parser_source': 'rule',
            'llm_latency': 0.0,
            'llm_success': False,
            'llm_error': '',
        }

        if bool(self.get_parameter('use_llm').value):
            begin = time.perf_counter()
            command, error = self.parse_with_llm(text)
            meta['llm_latency'] = round(time.perf_counter() - begin, 4)
            if command:
                meta['parser_source'] = 'llm'
                meta['llm_success'] = True
                return command, meta
            meta['parser_source'] = 'rule_fallback'
            meta['llm_error'] = error

        return self.parse_with_rules(text), meta

    def parse_with_llm(self, text: str) -> Tuple[Dict, str]:
        api_key_env = self.get_parameter('llm_api_key_env').value
        api_key = os.environ.get(api_key_env, '').strip()
        if not api_key:
            return {}, f'环境变量 {api_key_env} 未配置'

        payload = {
            'model': self.get_parameter('llm_model').value,
            'messages': [
                {'role': 'system', 'content': self.llm_system_prompt()},
                {'role': 'user', 'content': text},
            ],
            'temperature': 0.1,
        }
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            self.get_parameter('llm_api_url').value,
            data=body,
            headers={
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {api_key}',
            },
            method='POST',
        )

        try:
            timeout = float(self.get_parameter('llm_timeout').value)
            with urllib.request.urlopen(request, timeout=timeout) as response:
                result = json.loads(response.read().decode('utf-8'))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            return {}, f'LLM 请求失败: {exc}'

        try:
            content = result['choices'][0]['message']['content']
            command = self.normalize_llm_command(json.loads(self.extract_json_text(content)), text)
        except (KeyError, IndexError, TypeError, json.JSONDecodeError, ValueError) as exc:
            return {}, f'LLM 输出解析失败: {exc}'

        return command, ''

    def llm_system_prompt(self) -> str:
        targets = ', '.join(self.location_aliases.keys())
        return (
            '你是导盲机器人任务解析器。只输出 JSON，不要输出解释。'
            'JSON 字段必须包含 intent, targets, reply。'
            'intent 只能是 navigate, multi_navigate, cancel_navigation, unknown。'
            f'targets 只能从这些地点中选择: {targets}。'
            '如果用户要去一个地点，intent=navigate；多个地点按顺序导航，intent=multi_navigate；'
            '停止或取消导航，intent=cancel_navigation；无法识别地点，intent=unknown。'
        )

    def extract_json_text(self, content: str) -> str:
        match = re.search(r'\{.*\}', content, re.S)
        if not match:
            raise ValueError('未找到 JSON 对象')
        return match.group(0)

    def normalize_llm_command(self, command: Dict, raw_text: str) -> Dict:
        intent = command.get('intent', 'unknown')
        targets = command.get('targets', [])
        if isinstance(targets, str):
            targets = [targets]
        targets = [target for target in targets if target in self.location_aliases]

        if intent == 'cancel_navigation':
            targets = []
        elif intent == 'navigate' and len(targets) != 1:
            intent = 'unknown'
            targets = []
        elif intent == 'multi_navigate' and len(targets) < 2:
            intent = 'navigate' if len(targets) == 1 else 'unknown'
        elif intent not in ['navigate', 'multi_navigate', 'cancel_navigation', 'unknown']:
            intent = 'unknown'
            targets = []

        reply = command.get('reply') or self.default_reply(intent, targets)
        return {
            'intent': intent,
            'targets': targets,
            'raw_text': raw_text,
            'reply': reply,
        }

    def parse_with_rules(self, text: str) -> Dict:
        normalized = text.lower().replace(' ', '')

        if any(word in normalized for word in self.cancel_words):
            return {
                'intent': 'cancel_navigation',
                'targets': [],
                'raw_text': text,
                'reply': '好的，已为您取消当前导航任务。',
            }

        targets = self.extract_targets(normalized)
        if not targets:
            return {
                'intent': 'unknown',
                'targets': [],
                'raw_text': text,
                'reply': '抱歉，我没有识别出您想去的位置。请说例如：带我去302房间。',
            }

        intent = 'navigate' if len(targets) == 1 else 'multi_navigate'
        return {
            'intent': intent,
            'targets': targets,
            'raw_text': text,
            'reply': self.default_reply(intent, targets),
        }

    def extract_targets(self, text: str) -> List[str]:
        found = []
        for location, aliases in self.location_aliases.items():
            if any(alias.lower().replace(' ', '') in text for alias in aliases):
                found.append(location)

        numbered_rooms = re.findall(r'(?<!\d)(\d{3})(?!\d)', text)
        for room_number in numbered_rooms:
            location = f'room_{room_number}'
            if location in self.location_aliases and location not in found:
                found.append(location)

        return found

    def default_reply(self, intent: str, targets: List[str]) -> str:
        if intent == 'cancel_navigation':
            return '好的，已为您取消当前导航任务。'
        if intent == 'unknown' or not targets:
            return '抱歉，我没有识别出您想去的位置。请说例如：带我去302房间。'
        if len(targets) == 1:
            return f'好的，我将带您前往{self.display_name(targets[0])}。'
        target_names = '、'.join(self.display_name(target) for target in targets)
        return f'好的，我将依次带您前往{target_names}。'

    def display_name(self, target: str) -> str:
        display_names = {
            'entrance': '门口',
            'room_302': '302房间',
            'room_327': '327房间',
            'service_desk': '服务台',
            'living_room': '客厅',
            'kitchen': '厨房',
        }
        return display_names.get(target, target)

    def publish_json(self, publisher, payload: Dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
