import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
import random
import math
import time

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        self.obstacle_center = [0.0, 0.0, 0.0]
        self.obstacle_radius = 0.0
        self.workspace_radius = 1.0
        self.target_position = [0.0, 0.0, 0.0]
        self.starting_position = [0.0, 0.0, 0.0]
        self.acs_reference_point = [0.0, 0.0, 0.0]

        self.replan_requested = False
        self.last_button_state = 0

        self.timer = self.create_timer(1.0 / 25, self.plan_path)

        self.create_subscription(Float32MultiArray, '/obstacle_center', self.obstacle_center_callback, 10)
        self.create_subscription(Float32, '/obstacle_radius', self.obstacle_radius_callback, 10)
        self.create_subscription(Float32, '/workspace_radius', self.workspace_radius_callback, 10)
        self.create_subscription(Float32MultiArray, '/target_position', self.target_position_callback, 10)
        self.create_subscription(PoseStamped, '/admittance_controller/pose_debug', self.starting_position_callback, 10)
        # self.create_subscription(Joy, '/falcon0/buttons', self.joy_callback, 10) # Use this for falcon joystick
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(PoseStamped, '/ACS_reference_point', self.acs_reference_point_callback, 10)
        

        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.path_publisher = self.create_publisher(Float32MultiArray, '/rrt_path', 10) #TODO: clear up rrt_path & rrt_trajectory confusion (visualizer node publishes on trajectory)

        self.goal_sample_rate = 0.1
        self.step_size = 0.1
        self.max_iters = 5000

        self.distance_threshold = 0.05  # meters

    # Callback functions
    def obstacle_center_callback(self, msg: Float32MultiArray):
        self.obstacle_center = msg.data

    def obstacle_radius_callback(self, msg: Float32):
        self.obstacle_radius = msg.data

    def workspace_radius_callback(self, msg: Float32):
        self.workspace_radius = msg.data

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def acs_reference_point_callback(self, msg: PoseStamped):
        self.acs_reference_point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        distance_to_reference = self.distance(self.starting_position, self.acs_reference_point)

        # Request replan if the distance to the ACS reference point exceeds the threshold
        if distance_to_reference > self.distance_threshold:
            self.get_logger().info(f"Distance to ACS reference point is {distance_to_reference:.2f}, triggering replanning.")
            self.replan_requested = True

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > 2:
            current_button_state = msg.buttons[2] # Home button on wii remote

            # Request replanning on button press
            if current_button_state == 1 and self.last_button_state == 0:
                self.replan_requested = True
                # self.get_logger().info("Manual replanning requested.")
            self.last_button_state = current_button_state

    # RRT Algorithm and Path Planning
    def plan_path(self):
        if not self.replan_requested:
            return

        self.replan_requested = False  # Reset trigger

        if self.obstacle_radius > 0:
            obstacles = [(self.obstacle_center[0], self.obstacle_center[1], self.obstacle_center[2], self.obstacle_radius+0.05)] # Add small buffer to obstacle radius
        else:
            obstacles = []

        if self.is_point_in_obstacle(self.starting_position, obstacles) or self.is_point_in_obstacle(self.target_position, obstacles):
            # self.get_logger().info(f"Start or target position is inside an obstacle. Cannot plan path.")
            return

        start_time = time.perf_counter() #TODO: remove (debug)
        path, duration = self.build_rrt(
            self.starting_position,
            self.target_position,
            obstacles,
            self.workspace_radius,
            self.goal_sample_rate,
            self.step_size,
            self.max_iters
        )
        end_time = time.perf_counter() #TODO: remove (debug)

        if path:
            self.publish_path(path)
        # self.get_logger().info(f"RRT execution time: {end_time - start_time:.4f} seconds") #TODO: remove (debug)

    def distance(self, p1, p2):
        return math.sqrt(sum((p2[i] - p1[i])**2 for i in range(3)))

    def line_circle_collision(self, p1, p2, circle):
        cx, cy, cz, r = circle
        dx, dy, dz = [p2[i] - p1[i] for i in range(3)]
        fx, fy, fz = [p1[i] - circle[i] for i in range(3)]

        a = dx**2 + dy**2 + dz**2
        b = 2 * (fx * dx + fy * dy + fz * dz)
        c = fx**2 + fy**2 + fz**2 - r**2

        discriminant = b**2 - 4 * a * c
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
        while True:
            x = random.uniform(-radius, radius)
            y = random.uniform(-radius, radius)
            z = random.uniform(0, radius)
            if x**2 + y**2 + z**2 <= radius**2:
                return [x, y, z]

    def build_rrt(self, start, goal, obstacles, radius, goal_sample_rate, step_size=0.1, max_iters=5000):
        start_time = time.perf_counter()
        start_node = RRTTreeNode(start)
        nodes = [start_node]

        for _ in range(max_iters):
            rnd_point = goal if random.random() < goal_sample_rate else self.generate_random_point(radius)
            nearest = self.get_nearest_node(nodes, rnd_point)
            direction = [rnd_point[i] - nearest.point[i] for i in range(3)]
            norm = math.sqrt(sum(d**2 for d in direction))
            if norm == 0:
                continue
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

        # self.get_logger().warn("Max iterations reached.")
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

    def publish_path(self, path):
        path_msg = Float32MultiArray()
        path_msg.data = [coord for point in path for coord in point]
        self.path_publisher.publish(path_msg)
        # self.get_logger().info("Published path to /rrt_path")


class RRTTreeNode:
    def __init__(self, point, parent=None):
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
