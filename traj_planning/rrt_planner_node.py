import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, WrenchStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
import random
import math
import time

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        # Declare and get the "use_wii_controller" and "test" parameters
        self.declare_parameter('use_wii_controller', False)
        self.declare_parameter('test', True)
        self.use_wii_controller = self.get_parameter('use_wii_controller').get_parameter_value().bool_value
        self.test_mode = self.get_parameter('test').get_parameter_value().bool_value

        self.spherical_obstacles = []  # List of spherical obstacles
        self.cylinder_obstacle = None  # Cylinder obstacle (center, radius, height)
        self.workspace_radius = 1.0
        self.target_position = [0.0, 0.0, 0.0]
        self.starting_position = [0.0, 0.0, 0.0]
        self.acs_reference_point = [0.0, 0.0, 0.0]

        self.replan_requested = False
        self.last_button_state = 0

        # Variables for the new replanning condition
        self.non_cooperative = False
        self.ho_force = 0.0
        self.force_timer_start = None
        self.force_threshold_exceeded = False
        self.timer_duration = 1.0  # seconds
        self.ho_upper_bound = 7.0
        self.ho_lower_bound = 2.0

        self.timer = self.create_timer(1.0 / 25, self.plan_path)

        self.create_subscription(Float32MultiArray, '/spherical_obstacles', self.spherical_obstacles_callback, 10)
        self.create_subscription(Float32MultiArray, '/cylinder_base', self.cylinder_obstacle_callback, 10)
        self.create_subscription(Float32, '/workspace_radius', self.workspace_radius_callback, 10)
        self.create_subscription(Float32MultiArray, '/target_position', self.target_position_callback, 10)
        self.create_subscription(PoseStamped, '/admittance_controller/pose_debug', self.starting_position_callback, 10)
        self.create_subscription(PoseStamped, '/ACS_reference_point', self.acs_reference_point_callback, 10)
        self.create_subscription(Int32, '/differential_gt/decision', self.decision_callback, 10)
        self.create_subscription(WrenchStamped, '/differential_gt/wrench_from_ho', self.wrench_callback, 10)

        # Subscribe to the appropriate topic based on the parameter
        if self.use_wii_controller:
            self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        else:
            self.create_subscription(Joy, '/falcon0/buttons', self.joy_callback, 10)

        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.path_publisher = self.create_publisher(Float32MultiArray, '/rrt_path', 10)

        self.goal_sample_rate = 0.1
        self.step_size = 0.1
        self.max_iters = 5000

        self.distance_threshold = 0.05  # meters
        self.buffer = 0.01              # meters

        # Delay the first replan request by 2 seconds to ensure other nodes are active
        self.initial_replan_timer = self.create_timer(2.0, self.trigger_initial_replan)

        # Timer for continuous replanning
        self.continuous_replan_timer = None

    def trigger_initial_replan(self):
        self.replan_requested = True
        self.get_logger().info("Initial replan triggered.")
        
        # Cancel the timer after the first replan
        self.initial_replan_timer.cancel()

    def decision_callback(self, msg: Int32):
        self.non_cooperative = (msg.data == 1)

    def wrench_callback(self, msg: WrenchStamped):
        self.ho_force = math.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)

        if self.test_mode and self.non_cooperative and self.ho_force > self.ho_upper_bound:
            if self.force_timer_start is None:
                self.force_timer_start = time.time()
            elif time.time() - self.force_timer_start >= self.timer_duration:
                self.force_threshold_exceeded = True
                self.start_continuous_replanning()
        else:
            self.force_timer_start = None
            self.force_threshold_exceeded = False
            self.stop_continuous_replanning()

    def start_continuous_replanning(self):
        if self.continuous_replan_timer is None:
            self.continuous_replan_timer = self.create_timer(0.5, self.trigger_replan)
            self.get_logger().info("Continuous replanning started.")
            

    def stop_continuous_replanning(self):
        if self.continuous_replan_timer is not None:
            self.continuous_replan_timer.cancel()
            self.continuous_replan_timer = None
            self.get_logger().info("Continuous replanning stopped.")

    def trigger_replan(self):
        if self.ho_force < self.ho_lower_bound or not (self.test_mode and self.non_cooperative):
            self.stop_continuous_replanning()
        else:
            self.replan_requested = True

    # Callback functions
    def spherical_obstacles_callback(self, msg: Float32MultiArray):
        self.spherical_obstacles = []
        data = msg.data
        for i in range(0, len(data), 4):  # Each obstacle has 4 values: x, y, z, radius
            self.spherical_obstacles.append((data[i], data[i+1], data[i+2], data[i+3]))

    def cylinder_obstacle_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) == 5:  # Cylinder has 5 values: x, y, z (center), radius, height
            self.cylinder_obstacle = (data[0], data[1], data[2], data[3], data[4])

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
            # self.get_logger().info(f"Distance to ACS reference point is {distance_to_reference:.2f}, triggering replanning.")
            self.replan_requested = True

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > 2:
            # Use button index based on the parameter
            current_button_state = msg.buttons[2] if self.use_wii_controller else msg.buttons[1]

            # Request replanning on button press
            if current_button_state == 1 and self.last_button_state == 0:
                self.replan_requested = True
            self.last_button_state = current_button_state

    # RRT Algorithm and Path Planning
    def plan_path(self):
        if not self.replan_requested:
            return

        self.replan_requested = False  # Reset trigger

        obstacles = self.spherical_obstacles[:]
        if self.cylinder_obstacle:
            obstacles.append(self.cylinder_obstacle)

        if self.is_point_in_obstacle(self.starting_position, obstacles) or self.is_point_in_obstacle(self.target_position, obstacles):
            # self.get_logger().info(f"Start or target position is inside an obstacle. Cannot plan path.")
            return

        path, duration = self.build_rrt(
            self.starting_position,
            self.target_position,
            obstacles,
            self.workspace_radius,
            self.goal_sample_rate,
            self.step_size,
            self.max_iters
        )

        if path:
            self.publish_path(path)

    def distance(self, p1, p2):
        return math.sqrt(sum((p2[i] - p1[i])**2 for i in range(3)))

    def is_point_in_obstacle(self, point, obstacles):
        for obstacle in obstacles:
            if len(obstacle) == 4:  # Spherical obstacle
                ox, oy, oz, r = obstacle
                if (point[0] - ox)**2 + (point[1] - oy)**2 + (point[2] - oz)**2 <= r**2:
                    return True
            elif len(obstacle) == 5:  # Cylindrical obstacle
                cx, cy, cz, radius, height = obstacle
                dx, dy = point[0] - cx, point[1] - cy
                if dx**2 + dy**2 <= radius**2 and cz <= point[2] <= cz + height:
                    return True
        return False

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

    def line_cylinder_collision(self, p1, p2, cylinder):
        cx, cy, cz, radius, height = cylinder
        dx, dy, dz = [p2[i] - p1[i] for i in range(3)]
        fx, fy, fz = [p1[i] - cx if i < 2 else p1[i] - cz for i in range(3)]

        # Check for horizontal collision (cylinder's circular base)
        a = dx**2 + dy**2
        b = 2 * (fx * dx + fy * dy)
        c = fx**2 + fy**2 - radius**2

        discriminant = b**2 - 4 * a * c
        if discriminant >= 0:
            discriminant = math.sqrt(discriminant)
            t1 = (-b - discriminant) / (2 * a)
            t2 = (-b + discriminant) / (2 * a)

            # Check if the intersection points are within the cylinder's height
            for t in [t1, t2]:
                if 0 <= t <= 1:
                    z_intersection = p1[2] + t * dz
                    if cz <= z_intersection <= cz + height:
                        return True

        return False

    def is_collision_free(self, p1, p2, obstacles):
        for obstacle in obstacles:
            if len(obstacle) == 4:  # Spherical obstacle
                if self.line_circle_collision(p1, p2, obstacle):
                    return False
            elif len(obstacle) == 5:  # Cylindrical obstacle
                if self.line_cylinder_collision(p1, p2, obstacle):
                    return False
        return True

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
        i = 0  # Initialize i before the loop
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.is_collision_free(path[i], path[j], obstacles):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    def publish_path(self, path):
        path_msg = Float32MultiArray()
        path_msg.data = [coord for point in path for coord in point]
        self.path_publisher.publish(path_msg)

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
