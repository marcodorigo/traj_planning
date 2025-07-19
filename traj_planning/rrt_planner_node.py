import rclpy
from rclpy.node import Node
import random
import math
import time


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.timer = self.create_timer(1.0 / 50.0, self.plan_path)  # 50Hz frequency

        # Declare parameters
        self.declare_parameter('obstacle_center', [0.5, 0.15, 0.7])
        self.declare_parameter('obstacle_radius', 0.05)
        self.declare_parameter('workspace_radius', 0.85)
        self.declare_parameter('target_position', [0.7, 0.15, 0.5])

        # Get parameters
        self.obstacle_center = self.get_parameter('obstacle_center').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.workspace_radius = self.get_parameter('workspace_radius').value
        self.target_position = self.get_parameter('target_position').value


        # Parameters
        self.bounds = (0, 0, 85, 85)
        self.start = (1, 1)     # Start pos will be read from the robot's current position
        self.goal = (84, 84)    # Read from the target position parameter
        self.num_obstacles = 1

        # RRT parameters
        self.goal_sample_rate = 0.1
        self.step_size = 1.0
        self.max_iters = 5000

        self.get_logger().info("RRT Planner Node Initialized")

    def plan_path(self):
        obstacles = self.generate_random_obstacles(self.num_obstacles, self.bounds)

        if self.is_point_in_obstacle(self.start, obstacles) or self.is_point_in_obstacle(self.goal, obstacles):
            self.get_logger().warn("Start or goal inside obstacle. Skipping.")
            return

        path, duration = self.build_rrt(self.start, self.goal, obstacles, self.bounds, self.goal_sample_rate, self.step_size, self.max_iters)

        if path:
            self.get_logger().info(f"Path found in {duration:.4f} seconds.")
            self.plot_path(path, obstacles, self.start, self.goal, self.bounds)
        else:
            self.get_logger().warn("No path found.")

    # RRT Functions (same as original script)
    def distance(self, p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def line_circle_collision(self, p1, p2, circle):
        cx, cy, r = circle
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        fx = p1[0] - cx
        fy = p1[1] - cy

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        discriminant = math.sqrt(discriminant)

        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def is_collision_free(self, p1, p2, obstacles):
        return all(not self.line_circle_collision(p1, p2, c) for c in obstacles)

    def get_nearest_node(self, nodes, point):
        return min(nodes, key=lambda node: self.distance(node.point, point))

    def generate_random_point(self, bounds):
        x = random.uniform(bounds[0], bounds[2])
        y = random.uniform(bounds[1], bounds[3])
        return (x, y)

    def build_rrt(self, start, goal, obstacles, bounds, goal_sample_rate, step_size=1.0, max_iters=5000):
        start_time = time.perf_counter()

        start_node = Node(start)
        nodes = [start_node]

        for _ in range(max_iters):
            if random.random() < goal_sample_rate:
                rnd_point = goal
            else:
                rnd_point = self.generate_random_point(bounds)

            nearest = self.get_nearest_node(nodes, rnd_point)
            theta = math.atan2(rnd_point[1] - nearest.point[1], rnd_point[0] - nearest.point[0])
            new_point = (
                nearest.point[0] + step_size * math.cos(theta),
                nearest.point[1] + step_size * math.sin(theta),
            )

            if self.is_collision_free(nearest.point, new_point, obstacles):
                new_node = Node(new_point, nearest)
                nodes.append(new_node)

                if self.distance(new_point, goal) <= step_size and self.is_collision_free(new_point, goal, obstacles):
                    goal_node = Node(goal, new_node)
                    nodes.append(goal_node)
                    path = self.extract_path(goal_node)
                    end_time = time.perf_counter()
                    return self.smooth_path(path, obstacles), end_time - start_time

        self.get_logger().warn("Max iterations reached.")
        end_time = time.perf_counter()
        return None, end_time - start_time

    def extract_path(self, node):
        path = []
        while node:
            path.append(node.point)
            node = node.parent
        return path[::-1]

    def smooth_path(self, path, obstacles):
        if not path or len(path) < 3:
            return path

        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.is_collision_free(path[i], path[j], obstacles):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    def is_point_in_obstacle(self, point, obstacles):
        for ox, oy, r in obstacles:
            if (point[0] - ox)**2 + (point[1] - oy)**2 <= r**2:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()