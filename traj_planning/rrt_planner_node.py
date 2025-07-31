import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import random
import math
import time


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # Initialize parameters with default values
        self.obstacle_center = [0.0, 0.0, 0.0]
        self.obstacle_radius = 0.0
        self.workspace_radius = 1.0  # Default radius
        self.target_position = [0.0, 0.0, 0.0]  # 3D target position
        self.starting_position = [0.0, 0.0, 0.0]  # 3D starting position

        # Timer for planning
        self.timer = self.create_timer(1.0 / 5.0, self.plan_path)  # 5Hz frequency

        # Subscribers to parameter topics
        self.create_subscription(
            Float32MultiArray,
            '/obstacle_center',
            self.obstacle_center_callback,
            10
        )
        self.create_subscription(
            Float32,
            '/obstacle_radius',
            self.obstacle_radius_callback,
            10
        )
        self.create_subscription(
            Float32,
            '/workspace_radius',
            self.workspace_radius_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_position_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/admittance_controller/pose_debug',
            self.starting_position_callback,
            10
        )

        # RRT parameters
        self.goal_sample_rate = 0.1
        self.step_size = 0.1
        self.max_iters = 5000

        self.get_logger().info("✅ RRT Planner Node Initialized")

    def obstacle_center_callback(self, msg: Float32MultiArray):
        self.obstacle_center = msg.data
        self.get_logger().info(f"Updated obstacle_center: {self.obstacle_center}")

    def obstacle_radius_callback(self, msg: Float32):
        self.obstacle_radius = msg.data
        self.get_logger().info(f"Updated obstacle_radius: {self.obstacle_radius}")

    def workspace_radius_callback(self, msg: Float32):
        self.workspace_radius = msg.data
        self.get_logger().info(f"Updated workspace_radius: {self.workspace_radius}")

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]  # Extract x, y, z for 3D planning
        self.get_logger().info(f"Updated target_position: {self.target_position}")

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]  # Extract x, y, z
        self.get_logger().info(f"Updated starting_position: {self.starting_position}")

    def plan_path(self):
        obstacles = self.generate_random_obstacles(self.obstacle_center, self.obstacle_radius)

        if self.is_point_in_obstacle(self.starting_position, obstacles) or self.is_point_in_obstacle(self.target_position, obstacles):
            self.get_logger().warn("Starting position or target position inside obstacle. Skipping.")
            return

        path, duration = self.build_rrt(self.starting_position, self.target_position, obstacles, self.workspace_radius, self.goal_sample_rate, self.step_size, self.max_iters)

        if path:
            self.get_logger().info(f"Path found in {duration:.4f} seconds.")
            self.plot_path(path, obstacles, self.starting_position, self.target_position, self.workspace_radius)
        else:
            self.get_logger().warn("No path found.")

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

    def generate_random_point(self, radius):
        # Generate a random point within a sphere of given radius (workspace limit) centered in [0, 0, 0]
        while True:
            x = random.uniform(-radius, radius)
            y = random.uniform(-radius, radius)
            z = random.uniform(0, radius) # Limit Z to positive values (no points below the base)
            if x**2 + y**2 + z**2 <= radius**2:
                return [x, y, z]

    def build_rrt(self, start, goal, obstacles, radius, goal_sample_rate, step_size=0.1, max_iters=5000):
        start_time = time.perf_counter()

        start_node = Node(start)
        nodes = [start_node]

        for _ in range(max_iters):
            if random.random() < goal_sample_rate:
                rnd_point = goal
            else:
                rnd_point = self.generate_random_point(radius)

            nearest = self.get_nearest_node(nodes, rnd_point)
            direction = [rnd_point[i] - nearest.point[i] for i in range(3)]
            norm = math.sqrt(sum(d**2 for d in direction))
            direction = [d / norm for d in direction]
            new_point = [nearest.point[i] + step_size * direction[i] for i in range(3)]

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
        for ox, oy, oz, r in obstacles:
            if (point[0] - ox)**2 + (point[1] - oy)**2 + (point[2] - oz)**2 <= r**2:
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