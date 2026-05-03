import heapq
import json
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String

GridCell = Tuple[int, int]


class PlannerCompareNode(Node):
    def __init__(self):
        super().__init__('planner_compare_node')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('auto_plan_on_map', False)
        self.declare_parameter('default_start_x', 0.0)
        self.declare_parameter('default_start_y', 0.0)
        self.declare_parameter('default_goal_x', 2.0)
        self.declare_parameter('default_goal_y', 1.5)

        self.map: Optional[OccupancyGrid] = None
        self.has_auto_planned = False
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)

        self.create_subscription(OccupancyGrid, '/map', self.handle_map, 10)
        self.create_subscription(String, '/planner_compare_request', self.handle_request, 10)
        self.astar_pub = self.create_publisher(Path, '/astar_path', 10)
        self.dijkstra_pub = self.create_publisher(Path, '/dijkstra_path', 10)
        self.result_pub = self.create_publisher(String, '/planner_compare_result', 10)
        self.get_logger().info('路径规划对比节点已启动：A* + Dijkstra。')

    def handle_map(self, msg: OccupancyGrid):
        self.map = msg
        if not self.has_auto_planned and bool(self.get_parameter('auto_plan_on_map').value):
            self.has_auto_planned = True
            start = (float(self.get_parameter('default_start_x').value), float(self.get_parameter('default_start_y').value))
            goal = (float(self.get_parameter('default_goal_x').value), float(self.get_parameter('default_goal_y').value))
            self.compare(start, goal, 'auto_plan_on_map')

    def handle_request(self, msg: String):
        try:
            data = json.loads(msg.data)
            start = (float(data['start']['x']), float(data['start']['y']))
            goal = (float(data['goal']['x']), float(data['goal']['y']))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.publish_result({'success': False, 'error_type': 'invalid_request', 'error': str(exc)})
            return
        self.compare(start, goal, 'topic_request')

    def compare(self, start_world: Tuple[float, float], goal_world: Tuple[float, float], source: str):
        if self.map is None:
            self.publish_result({'success': False, 'error_type': 'map_not_ready'})
            return

        try:
            start = self.world_to_grid(*start_world)
            goal = self.world_to_grid(*goal_world)
        except ValueError as exc:
            self.publish_result({'success': False, 'error_type': 'pose_out_of_map', 'error': str(exc)})
            return

        if not self.is_free(start) or not self.is_free(goal):
            self.publish_result({'success': False, 'error_type': 'start_or_goal_not_free', 'start_cell': start, 'goal_cell': goal})
            return

        astar = self.run('astar', start, goal)
        dijkstra = self.run('dijkstra', start, goal)
        if astar['success']:
            self.astar_pub.publish(self.to_path(astar['path']))
        if dijkstra['success']:
            self.dijkstra_pub.publish(self.to_path(dijkstra['path']))

        self.publish_result({
            'success': astar['success'] or dijkstra['success'],
            'request_source': source,
            'start': {'x': start_world[0], 'y': start_world[1]},
            'goal': {'x': goal_world[0], 'y': goal_world[1]},
            'astar': self.metrics(astar),
            'dijkstra': self.metrics(dijkstra),
        })

    def run(self, algorithm: str, start: GridCell, goal: GridCell) -> Dict:
        begin = time.perf_counter()
        path, expanded = self.search(algorithm, start, goal)
        elapsed = time.perf_counter() - begin
        return {
            'success': bool(path),
            'path': path,
            'path_length': self.path_length(path),
            'planning_time': elapsed,
            'expanded_nodes': expanded,
        }

    def search(self, algorithm: str, start: GridCell, goal: GridCell) -> Tuple[List[GridCell], int]:
        heap = [(0.0, 0.0, start)]
        came_from: Dict[GridCell, Optional[GridCell]] = {start: None}
        cost: Dict[GridCell, float] = {start: 0.0}
        expanded = 0

        while heap:
            _, current_cost, current = heapq.heappop(heap)
            if current_cost > cost.get(current, float('inf')):
                continue
            expanded += 1
            if current == goal:
                return self.reconstruct(came_from, goal), expanded
            for neighbor, step_cost in self.neighbors(current):
                new_cost = cost[current] + step_cost
                if new_cost < cost.get(neighbor, float('inf')):
                    cost[neighbor] = new_cost
                    priority = new_cost + (self.heuristic(neighbor, goal) if algorithm == 'astar' else 0.0)
                    came_from[neighbor] = current
                    heapq.heappush(heap, (priority, new_cost, neighbor))
        return [], expanded

    def neighbors(self, cell: GridCell) -> List[Tuple[GridCell, float]]:
        base = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
        diag = [(-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]
        offsets = base + diag if self.allow_diagonal else base
        result = []
        for dx, dy, step in offsets:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if self.is_free(neighbor):
                result.append((neighbor, step))
        return result

    def reconstruct(self, came_from: Dict[GridCell, Optional[GridCell]], goal: GridCell) -> List[GridCell]:
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        return list(reversed(path))

    def world_to_grid(self, x: float, y: float) -> GridCell:
        info = self.map.info
        origin = info.origin.position
        cell = (int(math.floor((x - origin.x) / info.resolution)), int(math.floor((y - origin.y) / info.resolution)))
        if not self.in_bounds(cell):
            raise ValueError(f'坐标 ({x}, {y}) 超出地图范围')
        return cell

    def grid_to_world(self, cell: GridCell) -> Tuple[float, float]:
        info = self.map.info
        origin = info.origin.position
        return origin.x + (cell[0] + 0.5) * info.resolution, origin.y + (cell[1] + 0.5) * info.resolution

    def in_bounds(self, cell: GridCell) -> bool:
        return self.map is not None and 0 <= cell[0] < self.map.info.width and 0 <= cell[1] < self.map.info.height

    def is_free(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return False
        value = self.map.data[cell[1] * self.map.info.width + cell[0]]
        return (value >= 0 or not self.unknown_is_obstacle) and value < self.occupied_threshold

    def heuristic(self, cell: GridCell, goal: GridCell) -> float:
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1]) if self.allow_diagonal else abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    def path_length(self, path: List[GridCell]) -> float:
        if self.map is None or len(path) < 2:
            return 0.0
        total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))
        return total * self.map.info.resolution

    def to_path(self, cells: List[GridCell]) -> Path:
        msg = Path()
        msg.header.frame_id = self.map.header.frame_id or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for cell in cells:
            x, y = self.grid_to_world(cell)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def metrics(self, data: Dict) -> Dict:
        return {
            'success': data['success'],
            'path_length': round(float(data['path_length']), 4),
            'planning_time': round(float(data['planning_time']), 6),
            'expanded_nodes': int(data['expanded_nodes']),
            'path_points': len(data['path']),
        }

    def publish_result(self, payload: Dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerCompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
