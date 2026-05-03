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
        self.declare_parameter('safe_distance', 0.65)
        self.declare_parameter('critical_distance', 0.35)
        self.declare_parameter('forward_speed', 0.14)
        self.declare_parameter('slow_speed', 0.06)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('control_rate', 10.0)
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

        self.create_subscription(LaserScan, scan_topic, self.handle_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info('反应式避障节点已启动：订阅 /scan，发布 /cmd_vel。')

    def handle_scan(self, msg: LaserScan):
        self.latest_scan = msg
        front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.min_front_distance = self.min_range_in_sector(msg, -front_angle, front_angle)
        self.min_left_distance = self.min_range_in_sector(msg, front_angle, math.radians(100.0))
        self.min_right_distance = self.min_range_in_sector(msg, math.radians(-100.0), -front_angle)

    def control_loop(self):
        if not bool(self.get_parameter('enabled').value):
            return
        if self.latest_scan is None:
            return

        safe_distance = float(self.get_parameter('safe_distance').value)
        critical_distance = float(self.get_parameter('critical_distance').value)
        forward_speed = float(self.get_parameter('forward_speed').value)
        slow_speed = float(self.get_parameter('slow_speed').value)
        turn_speed = float(self.get_parameter('turn_speed').value)

        cmd = Twist()
        state = 'forward'

        if self.min_front_distance < critical_distance:
            state = 'critical_turn'
            cmd.linear.x = 0.0
            cmd.angular.z = self.choose_turn_direction(turn_speed)
        elif self.min_front_distance < safe_distance:
            state = 'avoidance_turn'
            cmd.linear.x = slow_speed
            cmd.angular.z = self.choose_turn_direction(turn_speed * 0.75)
        else:
            cmd.linear.x = forward_speed
            cmd.angular.z = 0.0

        was_active = self.avoidance_active
        self.avoidance_active = state in ['critical_turn', 'avoidance_turn']
        if self.avoidance_active and not was_active:
            self.trigger_count += 1

        self.cmd_pub.publish(cmd)
        self.publish_status(state, cmd)

    def choose_turn_direction(self, turn_speed: float) -> float:
        if self.min_left_distance >= self.min_right_distance:
            return turn_speed
        return -turn_speed

    def min_range_in_sector(self, scan: LaserScan, start_angle: float, end_angle: float) -> float:
        ranges = self.ranges_in_sector(scan, start_angle, end_angle)
        valid = [value for value in ranges if math.isfinite(value) and scan.range_min <= value <= scan.range_max]
        if not valid:
            return float('inf')
        return min(valid)

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

    def publish_status(self, state: str, cmd: Twist):
        payload = {
            'state': state,
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
        if math.isfinite(value):
            return round(float(value), 4)
        return None


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
