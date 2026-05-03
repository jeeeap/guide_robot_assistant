import csv
import json
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExperimentLoggerNode(Node):
    def __init__(self):
        super().__init__('experiment_logger_node')
        self.declare_parameter('log_file', '')
        self.declare_parameter('scenario_name', 'text_navigation_demo')

        self.scenario_name = self.get_parameter('scenario_name').value
        self.log_file = self.resolve_log_file()
        self.current_task: Optional[Dict] = None
        self.current_task_id = 0

        self.raw_text_subscription = self.create_subscription(String, '/raw_text', self.handle_raw_text, 10)
        self.task_status_subscription = self.create_subscription(String, '/task_status', self.handle_task_status, 10)
        self.navigation_status_subscription = self.create_subscription(
            String,
            '/navigation_status',
            self.handle_navigation_status,
            10,
        )

        self.ensure_csv_header()
        self.get_logger().info(f'实验记录节点已启动，CSV 文件: {self.log_file}')

    def resolve_log_file(self) -> Path:
        configured_path = self.get_parameter('log_file').value
        if configured_path:
            return Path(configured_path).expanduser()

        workspace_results = Path.home() / 'guide_robot_ws' / 'results'
        return workspace_results / 'experiment_log.csv'

    def ensure_csv_header(self):
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        if self.log_file.exists() and self.log_file.stat().st_size > 0:
            return

        with self.log_file.open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames())
            writer.writeheader()

    def fieldnames(self):
        return [
            'experiment_id',
            'scenario',
            'raw_text',
            'intent',
            'targets',
            'start_time',
            'end_time',
            'time_cost',
            'success',
            'final_state',
            'error_type',
            'action_status',
            'parser_source',
            'llm_latency',
            'llm_success',
            'llm_error',
        ]

    def handle_raw_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.current_task_id += 1
        self.current_task = {
            'experiment_id': self.current_task_id,
            'scenario': self.scenario_name,
            'raw_text': text,
            'intent': '',
            'targets': '',
            'start_time': time.time(),
            'end_time': '',
            'time_cost': '',
            'success': '',
            'final_state': 'received_text',
            'error_type': '',
            'action_status': '',
            'parser_source': '',
            'llm_latency': '',
            'llm_success': '',
            'llm_error': '',
        }
        self.get_logger().info(f'开始记录实验 {self.current_task_id}: {text}')

    def handle_task_status(self, msg: String):
        payload = self.safe_load_json(msg.data)
        if self.current_task is None or not payload:
            return

        command = payload.get('command', {})
        if command:
            self.current_task['intent'] = command.get('intent', '')
            self.current_task['targets'] = json.dumps(command.get('targets', []), ensure_ascii=False)
            self.current_task['final_state'] = payload.get('stage', 'parsed')
            self.current_task['parser_source'] = payload.get('parser_source', '')
            self.current_task['llm_latency'] = payload.get('llm_latency', '')
            self.current_task['llm_success'] = int(bool(payload.get('llm_success', False)))
            self.current_task['llm_error'] = payload.get('llm_error', '')

            if command.get('intent') == 'unknown':
                self.finish_task(success=False, final_state='parse_failed', error_type='unknown_target_or_intent')

    def handle_navigation_status(self, msg: String):
        payload = self.safe_load_json(msg.data)
        if self.current_task is None or not payload:
            return

        state = payload.get('state', '')
        self.current_task['final_state'] = state

        if 'action_status' in payload:
            self.current_task['action_status'] = payload.get('action_status')

        if state == 'failed':
            self.finish_task(success=False, final_state=state, error_type=payload.get('error', 'navigation_failed'))
        elif state == 'rejected':
            self.finish_task(success=False, final_state=state, error_type='goal_rejected')
        elif state == 'completed_all':
            self.finish_task(success=True, final_state=state, error_type='')
        elif state == 'result' and payload.get('action_status') not in [4, '4']:
            self.finish_task(success=False, final_state=state, error_type='nav2_result_not_succeeded')
        elif state == 'cancelled':
            self.finish_task(success=False, final_state=state, error_type='cancelled')

    def finish_task(self, success: bool, final_state: str, error_type: str):
        if self.current_task is None:
            return

        end_time = time.time()
        start_time = float(self.current_task.get('start_time', end_time))
        self.current_task['end_time'] = end_time
        self.current_task['time_cost'] = round(end_time - start_time, 3)
        self.current_task['success'] = int(success)
        self.current_task['final_state'] = final_state
        self.current_task['error_type'] = error_type

        with self.log_file.open('a', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames())
            writer.writerow(self.current_task)

        self.get_logger().info(
            f'实验 {self.current_task["experiment_id"]} 已记录: '
            f'success={self.current_task["success"]}, '
            f'time_cost={self.current_task["time_cost"]}, '
            f'final_state={final_state}'
        )
        self.current_task = None

    def safe_load_json(self, text: str) -> Dict:
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warning(f'收到非 JSON 状态消息: {text}')
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
