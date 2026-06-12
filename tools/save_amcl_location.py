#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def write_location(yaml_file, key, name, x, y, yaw):
    yaml_path = Path(yaml_file).expanduser()
    if yaml_path.exists():
        with yaml_path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    else:
        data = {}

    locations = data.setdefault('locations', {})
    old = locations.get(key, {})
    locations[key] = {
        'name': name or old.get('name') or key.replace('_', ' '),
        'x': float(x),
        'y': float(y),
        'yaw': float(yaw),
    }

    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with yaml_path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(description='Save the current /amcl_pose as a named navigation location.')
    parser.add_argument('yaml_file', help='Path to a locations YAML file.')
    parser.add_argument('location', help='Location key, for example kitchen or living_room.')
    parser.add_argument('--name', help='Human-readable display name. Defaults to the location key.')
    parser.add_argument('--topic', default='/amcl_pose')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('save_amcl_location')
    future = rclpy.task.Future()

    def callback(msg):
        if not future.done():
            future.set_result(msg)

    subscription = node.create_subscription(PoseWithCovarianceStamped, args.topic, callback, 10)
    try:
        while rclpy.ok() and not future.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        msg = future.result()
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        write_location(args.yaml_file, args.location, args.name, pose.position.x, pose.position.y, yaw)
        print(
            f'Updated {args.location}: '
            f'x={pose.position.x:.4f}, y={pose.position.y:.4f}, yaw={yaw:.4f}'
        )
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
