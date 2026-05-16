import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class NavClientNode(Node):
    def __init__(self):
        super().__init__('nav_client_node')
        self.declare_parameter('locations_file', '')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('nav2_wait_timeout_sec', 15.0)

        self.goal_frame_id = self.get_parameter('goal_frame_id').value
        action_name = self.get_parameter('action_name').value
        self.nav2_wait_timeout_sec = float(self.get_parameter('nav2_wait_timeout_sec').value)
        self.locations = self.load_locations()
        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.status_publisher = self.create_publisher(String, '/navigation_status', 10)
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        self.subscription = self.create_subscription(String, '/navigation_command', self.handle_command, 10)

        self.goal_queue: List[str] = []
        self.current_goal: Optional[str] = None
        self.current_goal_handle = None
        self.get_logger().info(f'导航客户端节点已启动，已加载 {len(self.locations)} 个目标点。')

    def load_locations(self) -> Dict:
        configured_path = self.get_parameter('locations_file').value
        if configured_path:
            locations_path = Path(configured_path).expanduser()
        else:
            package_share = get_package_share_directory('guide_robot_assistant')
            locations_path = Path(package_share) / 'config' / 'task_locations.yaml'

        if not locations_path.exists():
            raise FileNotFoundError(f'目标点配置文件不存在: {locations_path}')

        with locations_path.open('r', encoding='utf-8') as file:
            data = yaml.safe_load(file) or {}

        locations = data.get('locations', {})
        if not locations:
            self.get_logger().warning('目标点配置为空，请检查 task_locations.yaml。')
        return locations

    def handle_command(self, msg: String):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status('invalid_command', error=f'JSON 解析失败: {exc}')
            return

        intent = command.get('intent')
        targets = command.get('targets', [])

        if intent == 'cancel_navigation':
            self.cancel_current_goal()
            return

        if intent not in ['navigate', 'multi_navigate']:
            self.publish_status('ignored', intent=intent, reason='不是导航任务')
            return

        if isinstance(targets, str):
            targets = [targets]

        unknown_targets = [target for target in targets if target not in self.locations]
        if unknown_targets:
            text = f'目标点不存在: {unknown_targets}。请检查 task_locations.yaml。'
            self.get_logger().warning(text)
            self.publish_status('failed', error=text, targets=targets)
            self.publish_tts('抱歉，这个目标点还没有配置地图坐标。')
            return

        self.goal_queue = list(targets)
        if self.current_goal_handle is not None:
            self.cancel_current_goal(start_next_after_cancel=True)
        else:
            self.start_next_goal()

    def start_next_goal(self):
        if not self.goal_queue:
            self.current_goal = None
            self.publish_status('completed_all')
            self.publish_tts('导航任务已完成。')
            return

        target = self.goal_queue.pop(0)
        location = self.locations[target]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(location)

        self.current_goal = target
        self.publish_status('waiting_for_nav2', target=target)
        self.get_logger().info(
            f'正在等待 Nav2 NavigateToPose action 服务，最多等待 {self.nav2_wait_timeout_sec:.1f} 秒...'
        )
        if not self.action_client.wait_for_server(timeout_sec=self.nav2_wait_timeout_sec):
            self.goal_queue.insert(0, target)
            self.current_goal = None
            text = (
                'Nav2 NavigateToPose action 服务不可用。'
                '请确认 tb3_simulation_launch.py 已正常启动、机器人已生成、'
                '并且 /tf 中存在 map、odom、base_link 坐标变换。'
            )
            self.get_logger().error(text)
            self.publish_status('nav2_unavailable', target=target, error=text)
            self.publish_tts('导航系统还没有准备好，请先检查仿真和导航服务。')
            return

        self.publish_status('goal_sent', target=target, pose=location)
        self.publish_tts(f'开始导航到{location.get("name", target)}。')
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def create_pose(self, location: Dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(location.get('x', 0.0))
        pose.pose.position.y = float(location.get('y', 0.0))
        pose.pose.position.z = float(location.get('z', 0.0))

        yaw = float(location.get('yaw', 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            target = self.current_goal
            self.current_goal_handle = None
            self.publish_status('rejected', target=target)
            self.publish_tts('导航目标被系统拒绝。')
            self.start_next_goal()
            return

        self.current_goal_handle = goal_handle
        self.publish_status('accepted', target=self.current_goal)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        if distance is not None:
            self.publish_status('feedback', target=self.current_goal, distance_remaining=float(distance))

    def result_callback(self, future):
        result = future.result()
        target = self.current_goal
        status = result.status
        self.current_goal_handle = None
        self.publish_status('result', target=target, action_status=int(status))

        if status == 4:
            self.publish_tts(f'已到达{self.locations[target].get("name", target)}。')
        else:
            self.publish_tts(f'前往{self.locations[target].get("name", target)}的导航任务未成功完成。')

        self.start_next_goal()

    def cancel_current_goal(self, start_next_after_cancel: bool = False):
        self.goal_queue = [] if not start_next_after_cancel else self.goal_queue
        if self.current_goal_handle is None:
            self.current_goal = None
            self.publish_status('cancelled', reason='当前没有正在执行的导航目标')
            self.publish_tts('当前没有正在执行的导航任务。')
            if start_next_after_cancel:
                self.start_next_goal()
            return

        target = self.current_goal
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.after_cancel(target, start_next_after_cancel))

    def after_cancel(self, target: str, start_next_after_cancel: bool):
        self.publish_status('cancelled', target=target)
        self.publish_tts('已取消当前导航任务。')
        self.current_goal = None
        self.current_goal_handle = None
        if start_next_after_cancel:
            self.start_next_goal()

    def publish_status(self, state: str, **kwargs):
        payload = {'state': state}
        payload.update(kwargs)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(msg)
        self.get_logger().info(msg.data)

    def publish_tts(self, text: str):
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
