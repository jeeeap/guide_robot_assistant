import json
import math
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ReactiveAvoidanceNode(Node):
    def __init__(self):
        super().__init__('reactive_avoidance_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/avoidance_status')
        self.declare_parameter('front_angle_deg', 50.0)
        self.declare_parameter('side_angle_deg', 90.0)
        # 4-level distance thresholds (盲人场景需要更大的预警距离)
        self.declare_parameter('warning_distance', 1.2)    # 预警：前方有障碍
        self.declare_parameter('safe_distance', 0.65)      # 减速避让
        self.declare_parameter('critical_distance', 0.35)  # 紧急原地转向
        self.declare_parameter('side_warning_distance', 0.45)  # 侧方障碍预警
        # Speeds
        self.declare_parameter('forward_speed', 0.14)
        self.declare_parameter('warning_speed', 0.09)      # 预警区减速
        self.declare_parameter('slow_speed', 0.05)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('control_rate', 10.0)
        # TTS
        self.declare_parameter('tts_cooldown_sec', 3.0)    # 同一消息最小间隔
        self.declare_parameter('enabled', True)

        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value
        control_rate = float(self.get_parameter('control_rate').value)

        self.latest_scan: Optional[LaserScan] = None
        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')
        self.avoidance_active = False
        self.trigger_count = 0
        self.start_time = time.time()
        self.current_state = 'clear'
        self._last_tts_time: float = 0.0
        self._last_tts_text: str = ''

        self.create_subscription(LaserScan, scan_topic, self.handle_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info(
            '增强版反应式避障节点已启动（4级预警+方向TTS）：'
            f'预警={self.get_parameter("warning_distance").value}m，'
            f'避让={self.get_parameter("safe_distance").value}m，'
            f'紧急={self.get_parameter("critical_distance").value}m。'
        )

    def handle_scan(self, msg: LaserScan):
        self.latest_scan = msg
        front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.min_front_distance = self.min_range_in_sector(msg, -front_angle, front_angle)
        self.min_left_distance = self.min_range_in_sector(msg, front_angle, side_angle)
        self.min_right_distance = self.min_range_in_sector(msg, -side_angle, -front_angle)

    def control_loop(self):
        if not bool(self.get_parameter('enabled').value):
            return
        if self.latest_scan is None:
            return

        warning_dist = float(self.get_parameter('warning_distance').value)
        safe_dist = float(self.get_parameter('safe_distance').value)
        critical_dist = float(self.get_parameter('critical_distance').value)
        side_warn_dist = float(self.get_parameter('side_warning_distance').value)
        forward_speed = float(self.get_parameter('forward_speed').value)
        warning_speed = float(self.get_parameter('warning_speed').value)
        slow_speed = float(self.get_parameter('slow_speed').value)
        turn_speed = float(self.get_parameter('turn_speed').value)

        cmd = Twist()
        prev_state = self.current_state

        if self.min_front_distance < critical_dist:
            self.current_state = 'critical_turn'
            cmd.linear.x = 0.0
            cmd.angular.z = self.choose_turn_direction(turn_speed)
        elif self.min_front_distance < safe_dist:
            self.current_state = 'avoidance_turn'
            cmd.linear.x = slow_speed
            cmd.angular.z = self.choose_turn_direction(turn_speed * 0.75)
        elif self.min_front_distance < warning_dist:
            self.current_state = 'warning'
            cmd.linear.x = warning_speed
            cmd.angular.z = 0.0
        else:
            self.current_state = 'clear'
            cmd.linear.x = forward_speed
            cmd.angular.z = 0.0

        was_active = self.avoidance_active
        self.avoidance_active = self.current_state in ['critical_turn', 'avoidance_turn']
        if self.avoidance_active and not was_active:
            self.trigger_count += 1

        self._handle_tts(prev_state, side_warn_dist)
        self.cmd_pub.publish(cmd)
        self.publish_status(cmd)

    def _handle_tts(self, prev_state: str, side_warn_dist: float):
        message = ''

        if self.current_state != prev_state:
            if self.current_state == 'critical_turn':
                message = '紧急避障，请注意安全！'
            elif self.current_state == 'avoidance_turn':
                if self.min_left_distance >= self.min_right_distance:
                    message = '前方有障碍，正在向左避让，请稍候。'
                else:
                    message = '前方有障碍，正在向右避让，请稍候。'
            elif self.current_state == 'warning':
                dist_m = round(self.min_front_distance, 1)
                message = f'注意，前方约{dist_m}米处有障碍物，已减速。'
            elif self.current_state == 'clear' and prev_state != 'clear':
                message = '前方已清空，继续前进。'

        # 侧方障碍独立预警（仅在 clear 状态下避免与前方预警叠加）
        if not message and self.current_state == 'clear':
            left_blocked = self.min_left_distance < side_warn_dist
            right_blocked = self.min_right_distance < side_warn_dist
            if left_blocked and not right_blocked:
                message = '左侧有障碍物，请小心。'
            elif right_blocked and not left_blocked:
                message = '右侧有障碍物，请小心。'

        if not message:
            return

        now = time.time()
        cooldown = float(self.get_parameter('tts_cooldown_sec').value)
        # 同一文本有冷却，不同文本立即播报（状态切换时）
        if message == self._last_tts_text and now - self._last_tts_time < cooldown:
            return

        tts_msg = String()
        tts_msg.data = message
        self.tts_pub.publish(tts_msg)
        self._last_tts_time = now
        self._last_tts_text = message
        self.get_logger().info(f'[避障TTS] {message}')

    def choose_turn_direction(self, turn_speed: float) -> float:
        if self.min_left_distance >= self.min_right_distance:
            return turn_speed
        return -turn_speed

    def min_range_in_sector(self, scan: LaserScan, start_angle: float, end_angle: float) -> float:
        ranges = self.ranges_in_sector(scan, start_angle, end_angle)
        valid = [v for v in ranges if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
        return min(valid) if valid else float('inf')

    def ranges_in_sector(self, scan: LaserScan, start_angle: float, end_angle: float) -> List[float]:
        if scan.angle_increment == 0.0:
            return []
        start_index = self.angle_to_index(scan, start_angle)
        end_index = self.angle_to_index(scan, end_angle)
        start_index = max(0, min(start_index, len(scan.ranges) - 1))
        end_index = max(0, min(end_index, len(scan.ranges) - 1))
        if start_index <= end_index:
            return list(scan.ranges[start_index:end_index + 1])
        return list(scan.ranges[end_index:start_index + 1])

    def angle_to_index(self, scan: LaserScan, angle: float) -> int:
        return int(round((angle - scan.angle_min) / scan.angle_increment))

    def publish_status(self, cmd: Twist):
        payload = {
            'state': self.current_state,
            'avoidance_active': self.avoidance_active,
            'trigger_count': self.trigger_count,
            'min_front_distance': self.safe_float(self.min_front_distance),
            'min_left_distance': self.safe_float(self.min_left_distance),
            'min_right_distance': self.safe_float(self.min_right_distance),
            'linear_x': round(float(cmd.linear.x), 4),
            'angular_z': round(float(cmd.angular.z), 4),
            'elapsed_time': round(time.time() - self.start_time, 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def safe_float(self, value: float):
        return round(float(value), 4) if math.isfinite(value) else None


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
