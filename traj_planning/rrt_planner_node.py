import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
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

        # Publisher for visualizing the goal point in RViz
        self.marker_publisher = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

        # Publisher for the computed path
        self.path_publisher = self.create_publisher(
            Float32MultiArray,
            '/rrt_path',
            10
        )

        # RRT parameters
        self.goal_sample_rate = 0.1
        self.step_size = 0.1
        self.max_iters = 5000

        self.get_logger().info("RRT Planner Node Initialized")

    def obstacle_center_callback(self, msg: Float32MultiArray):
        self.obstacle_center = msg.data
        #self.get_logger().info(f"Updated obstacle_center: {self.obstacle_center}")

    def obstacle_radius_callback(self, msg: Float32):
        self.obstacle_radius = msg.data
        #self.get_logger().info(f"Updated obstacle_radius: {self.obstacle_radius}")

    def workspace_radius_callback(self, msg: Float32):
        self.workspace_radius = msg.data
        #self.get_logger().info(f"Updated workspace_radius: {self.workspace_radius}")

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]  # Extract x, y, z for 3D planning
        #self.get_logger().info(f"Updated target_position: {self.target_position}")
        self.publish_goal_marker()  # Publish the goal marker

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]  # Extract x, y, z
        #self.get_logger().info(f"Updated starting_position: {self.starting_position}")

    def plan_path(self):
        # Use the obstacle center and radius directly
        if self.obstacle_radius > 0:
            obstacles = [(self.obstacle_center[0], self.obstacle_center[1], self.obstacle_center[2], self.obstacle_radius)]
        else:
            obstacles = []

        # Check if the starting or target position is inside an obstacle
        if self.is_point_in_obstacle(self.starting_position, obstacles) or self.is_point_in_obstacle(self.target_position, obstacles):
            self.get_logger().info(f"Start or target position is inside an obstacle. Cannot plan path.")
            return

        # Build the RRT path
        start_time = time.perf_counter()  # Start timing
        path, duration = self.build_rrt(
            self.starting_position,
            self.target_position,
            obstacles,
            self.workspace_radius,
            self.goal_sample_rate,
            self.step_size,
            self.max_iters
        )
        end_time = time.perf_counter()  # End timing

        # Publish the path if found
        if path:
            self.publish_path(path)
        # Log the execution time
        self.get_logger().info(f"RRT execution time: {end_time - start_time:.4f} seconds")

    def distance(self, p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def line_circle_collision(self, p1, p2, circle):
        cx, cy, cz, r = circle  # Unpack the 3D center and radius
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        fx = p1[0] - cx
        fy = p1[1] - cy
        fz = p1[2] - cz

        a = dx * dx + dy * dy + dz * dz
        b = 2 * (fx * dx + fy * dy + fz * dz)
        c = fx * fx + fy * fy + fz * fz - r * r

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

        # Use the custom RRTTreeNode class
        start_node = RRTTreeNode(start)
        nodes = [start_node]

        for _ in range(max_iters):
            if random.random() < goal_sample_rate:
                rnd_point = goal
            else:
                rnd_point = self.generate_random_point(radius)

            nearest = self.get_nearest_node(nodes, rnd_point)
            direction = [rnd_point[i] - nearest.point[i] for i in range(3)]
            norm = math.sqrt(sum(d**2 for d in direction))

            # Check if norm is zero to avoid division by zero
            if norm == 0:
                continue  # Skip this iteration

            direction = [d / norm for d in direction]
            new_point = [nearest.point[i] + step_size * direction[i] for i in range(3)]

            if self.is_collision_free(nearest.point, new_point, obstacles):
                new_node = RRTTreeNode(new_point, nearest)
                nodes.append(new_node)

                if self.distance(new_point, goal) <= step_size and self.is_collision_free(new_point, goal, obstacles):
                    goal_node = RRTTreeNode(goal, new_node)
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

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"  # Replace with your robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.target_position[0]
        marker.pose.position.y = self.target_position[1]
        marker.pose.position.z = self.target_position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Adjust size as needed
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        self.marker_publisher.publish(marker)

    # Add a method to publish the path
    def publish_path(self, path):
        path_msg = Float32MultiArray()
        # Flatten the path (list of points) into a single list
        path_msg.data = [coord for point in path for coord in point]
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published path to /rrt_path")


class RRTTreeNode:
    def __init__(self, point, parent=None):
        """
        Represents a node in the RRT tree.
        :param point: The 3D coordinates of the node.
        :param parent: The parent node in the tree.
        """
        self.point = point
        self.parent = parent


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()