import json
import os
import re
import time
import urllib.error
import urllib.request
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CenterNode(Node):
    # 保留最近 N 轮对话传给 LLM（每轮 = 1条user + 1条assistant）
    MAX_HISTORY_TURNS = 5

    def __init__(self):
        super().__init__('center_node')
        self.declare_parameter('use_llm', False)
        self.declare_parameter('llm_api_url', 'https://open.bigmodel.cn/api/paas/v4/chat/completions')
        self.declare_parameter('llm_model', 'glm-4-flash')
        self.declare_parameter('llm_api_key_env', 'ZHIPUAI_API_KEY')
        self.declare_parameter('llm_timeout', 8.0)
        self.declare_parameter('enable_memory', True)

        self.command_publisher = self.create_publisher(String, '/navigation_command', 10)
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        self.status_publisher = self.create_publisher(String, '/task_status', 10)
        self.memory_publisher = self.create_publisher(String, '/agent_memory', 10)

        self.subscription = self.create_subscription(String, '/raw_text', self.handle_text, 10)
        self.nav_status_sub = self.create_subscription(
            String, '/navigation_status', self._handle_nav_status, 10
        )

        self.location_aliases: Dict[str, List[str]] = {
            'entrance': ['门口', '入口', '大门', 'entrance'],
            'room_302': ['302', '302房间', '三零二', 'room302', 'room_302'],
            'room_327': ['327', '327房间', '三二七', 'room327', 'room_327'],
            'service_desk': ['服务台', '前台', '咨询台', '桌子', 'desk', 'service_desk'],
            'living_room': ['客厅', '大厅', 'living_room'],
            'kitchen': ['厨房', 'kitchen'],
        }
        self.cancel_words = ['停止', '取消', '别去了', '暂停', '取消导航', 'stop', 'cancel', '算了', '不去了']
        self.again_words = ['再去一次', '再去', '还去', '重新去', '再来一次', '再导航']
        self.back_words = ['回去', '回来', '回到起点', '回门口', '回到门口', '回到入口', '回出发点']
        self.where_words = ['我在哪', '现在在哪', '到了吗', '到哪了', '在哪里', '位置']

        # Agent 记忆：位置历史 + 对话历史
        self.last_visited: Deque[str] = deque(maxlen=10)
        self.current_location: Optional[str] = None
        self.conversation_history: List[Dict] = []  # LLM message pairs

        self.get_logger().info('中心调度节点（Agentic版）已启动，等待 /raw_text 指令。')
        self._publish_memory()

    # ── 导航状态回调，更新位置记忆 ─────────────────────────────────────────
    def _handle_nav_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = payload.get('state', '')
        target = payload.get('target')

        if state == 'result' and payload.get('action_status') in [4, '4'] and target:
            self.last_visited.appendleft(target)
            self.current_location = target
            self._publish_memory()
            self.get_logger().info(f'[记忆] 已到达 {target}，位置记忆已更新。')
        elif state == 'goal_sent' and target:
            self.current_location = f'前往{self.display_name(target)}'
            self._publish_memory()

    # ── 主处理逻辑 ─────────────────────────────────────────────────────────
    def handle_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        command, meta = self.parse_command(text)

        # 更新对话历史（用于下一轮 LLM 上下文）
        if bool(self.get_parameter('enable_memory').value):
            reply = command.get('reply', '好的。')
            self.conversation_history.append({'role': 'user', 'content': text})
            self.conversation_history.append({'role': 'assistant', 'content': reply})
            max_msgs = self.MAX_HISTORY_TURNS * 2
            if len(self.conversation_history) > max_msgs:
                self.conversation_history = self.conversation_history[-max_msgs:]

        self.publish_json(self.status_publisher, {
            'stage': 'parsed',
            'raw_text': text,
            'command': command,
            'parser_source': meta['parser_source'],
            'llm_latency': meta['llm_latency'],
            'llm_success': meta['llm_success'],
            'llm_error': meta['llm_error'],
            'memory': {
                'current_location': self.current_location,
                'last_visited': list(self.last_visited)[:3],
                'history_turns': len(self.conversation_history) // 2,
            },
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

    # ── 解析入口：LLM优先，规则兜底 ───────────────────────────────────────
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

    # ── LLM 解析（携带对话历史）────────────────────────────────────────────
    def parse_with_llm(self, text: str) -> Tuple[Dict, str]:
        api_key = os.environ.get(self.get_parameter('llm_api_key_env').value, '').strip()
        if not api_key:
            return {}, f'环境变量 {self.get_parameter("llm_api_key_env").value} 未配置'

        messages = [{'role': 'system', 'content': self.llm_system_prompt()}]
        # 加入历史上下文（排除最后一条 user，避免重复），用于多轮对话
        history_to_send = self.conversation_history[:-1] if self.conversation_history else []
        messages.extend(history_to_send)
        messages.append({'role': 'user', 'content': text})

        payload = {
            'model': self.get_parameter('llm_model').value,
            'messages': messages,
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
        ctx_parts = []
        if self.current_location:
            ctx_parts.append(f'机器人当前位置: {self.current_location}')
        if self.last_visited:
            visited_names = [self.display_name(loc) for loc in list(self.last_visited)[:3]]
            ctx_parts.append(f'最近访问: {", ".join(visited_names)}')
        context_str = ('；'.join(ctx_parts) + '。') if ctx_parts else ''

        return (
            '你是导盲机器人任务解析器，支持多轮对话和记忆。只输出 JSON，不要输出任何解释。'
            'JSON 字段必须包含 intent, targets, reply。'
            f'intent 可选值: navigate（去一个地点）, multi_navigate（按顺序去多个地点）, '
            'cancel_navigation（取消/停止）, query_location（询问当前位置）, unknown（无法理解）。'
            f'targets 只能从以下地点选择: {targets}。'
            '如果用户说"再去一次""还去""重新去"，根据对话历史推断上次目标填入targets。'
            '如果用户说"回去""回来""回门口"，targets=["entrance"]。'
            '如果用户询问当前在哪，intent=query_location，targets=[]。'
            f'{context_str}'
            'reply 字段用简短中文给出语音回复，语气自然亲切。'
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
        targets = [t for t in targets if t in self.location_aliases]

        if intent == 'cancel_navigation':
            targets = []
        elif intent == 'query_location':
            targets = []
        elif intent == 'navigate' and len(targets) != 1:
            intent = 'unknown' if not targets else 'navigate'
            targets = targets[:1]
        elif intent == 'multi_navigate' and len(targets) < 2:
            intent = 'navigate' if len(targets) == 1 else 'unknown'
        elif intent not in ('navigate', 'multi_navigate', 'cancel_navigation', 'query_location', 'unknown'):
            intent = 'unknown'
            targets = []

        reply = command.get('reply') or self.default_reply(intent, targets)
        return {'intent': intent, 'targets': targets, 'raw_text': raw_text, 'reply': reply}

    # ── 规则解析（带记忆的 Agentic 规则）──────────────────────────────────
    def parse_with_rules(self, text: str) -> Dict:
        normalized = text.lower().replace(' ', '')

        if any(w in normalized for w in self.cancel_words):
            return self._cmd('cancel_navigation', [], text, '好的，已为您取消当前导航任务。')

        if bool(self.get_parameter('enable_memory').value):
            # "再去一次" → 重复上一个目标
            if any(w in normalized for w in self.again_words):
                if self.last_visited:
                    target = list(self.last_visited)[0]
                    return self._cmd('navigate', [target], text,
                                     f'好的，再次带您前往{self.display_name(target)}。')
                return self._cmd('unknown', [], text, '抱歉，我还没有到达过任何地点，请告诉我您想去哪里。')

            # "回去" → 返回门口（起点）
            if any(w in normalized for w in self.back_words):
                return self._cmd('navigate', ['entrance'], text, '好的，带您返回门口。')

            # "在哪" → 查询当前位置
            if any(w in normalized for w in self.where_words):
                loc = self.current_location
                if loc and loc in self.location_aliases:
                    reply = f'您现在在{self.display_name(loc)}。'
                elif loc:
                    reply = f'机器人状态：{loc}。'
                else:
                    reply = '当前位置未知，可能还未开始导航。'
                return self._cmd('query_location', [], text, reply)

        targets = self.extract_targets(normalized)
        if not targets:
            return self._cmd('unknown', [], text,
                             '抱歉，我没有识别出您想去的位置。请说例如：带我去302房间。')

        intent = 'navigate' if len(targets) == 1 else 'multi_navigate'
        return self._cmd(intent, targets, text, self.default_reply(intent, targets))

    def extract_targets(self, text: str) -> List[str]:
        found = []
        for location, aliases in self.location_aliases.items():
            if any(alias.lower().replace(' ', '') in text for alias in aliases):
                found.append(location)

        for room_number in re.findall(r'(?<!\d)(\d{3})(?!\d)', text):
            location = f'room_{room_number}'
            if location in self.location_aliases and location not in found:
                found.append(location)

        return found

    # ── 辅助方法 ────────────────────────────────────────────────────────────
    def _cmd(self, intent: str, targets: List[str], raw_text: str, reply: str) -> Dict:
        return {'intent': intent, 'targets': targets, 'raw_text': raw_text, 'reply': reply}

    def default_reply(self, intent: str, targets: List[str]) -> str:
        if intent == 'cancel_navigation':
            return '好的，已为您取消当前导航任务。'
        if intent == 'query_location':
            loc = self.current_location
            if loc and loc in self.location_aliases:
                return f'您现在在{self.display_name(loc)}。'
            return '当前位置未知。'
        if intent == 'unknown' or not targets:
            return '抱歉，我没有识别出您想去的位置。请说例如：带我去302房间。'
        if len(targets) == 1:
            return f'好的，我将带您前往{self.display_name(targets[0])}。'
        target_names = '、'.join(self.display_name(t) for t in targets)
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

    def _publish_memory(self):
        self.publish_json(self.memory_publisher, {
            'current_location': self.current_location,
            'last_visited': list(self.last_visited),
            'history_turns': len(self.conversation_history) // 2,
        })


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
