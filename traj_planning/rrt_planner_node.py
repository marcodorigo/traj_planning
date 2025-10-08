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
        # Add parameter for obstacle safety buffer
        self.declare_parameter('obstacle_buffer', 0.05)
        
        self.use_wii_controller = self.get_parameter('use_wii_controller').get_parameter_value().bool_value
        self.test_mode = self.get_parameter('test').get_parameter_value().bool_value
        self.obstacle_buffer = self.get_parameter('obstacle_buffer').get_parameter_value().double_value

        self.spherical_obstacles = []  # List of spherical obstacles
        self.cylindrical_obstacles = []  # List of cylindrical obstacles
        self.cylinder_obstacle = None  # Cylinder base obstacle (center, radius, height)
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
        self.create_subscription(Float32MultiArray, '/cylindrical_obstacles', self.cylindrical_obstacles_callback, 10)
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

        self.distance_threshold = 0.1  # meters (was 0.05)
        
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
        self.get_logger().debug(f"Updated spherical obstacles: {self.spherical_obstacles}")

    def cylindrical_obstacles_callback(self, msg: Float32MultiArray):
        self.cylindrical_obstacles = []
        data = msg.data
        for i in range(0, len(data), 5):  # Each obstacle has 5 values: x, y, z, radius, height
            self.cylindrical_obstacles.append((data[i], data[i+1], data[i+2], data[i+3], data[i+4]))
        self.get_logger().debug(f"Updated cylindrical obstacles: {self.cylindrical_obstacles}")

    def cylinder_obstacle_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) == 5:  # Cylinder has 5 values: x, y, z (center), radius, height
            self.cylinder_obstacle = (data[0], data[1], data[2], data[3], data[4])
        self.get_logger().debug(f"Updated cylinder base obstacle: {self.cylinder_obstacle}")

    def workspace_radius_callback(self, msg: Float32):
        self.workspace_radius = msg.data
        self.get_logger().debug(f"Updated workspace radius: {self.workspace_radius}")

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]
        self.get_logger().debug(f"Updated target position: {self.target_position}")

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.get_logger().debug(f"Updated starting position: {self.starting_position}")

    def acs_reference_point_callback(self, msg: PoseStamped):
        self.acs_reference_point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        distance_to_reference = self.distance(self.starting_position, self.acs_reference_point)
        self.get_logger().debug(f"Distance to ACS reference point: {distance_to_reference}")

        # Request replan if the distance to the ACS reference point exceeds the threshold
        if distance_to_reference > self.distance_threshold:
            self.get_logger().info(f"Distance to ACS reference point exceeded threshold. Triggering replanning.")
            self.replan_requested = True

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > 2:
            # Use button index based on the parameter
            current_button_state = msg.buttons[2] if self.use_wii_controller else msg.buttons[1]

            # Request replanning on button press
            if current_button_state == 1 and self.last_button_state == 0:
                self.replan_requested = True
                self.get_logger().info("Replan requested via joystick button.")
            self.last_button_state = current_button_state

    # RRT Algorithm and Path Planning
    def plan_path(self):
        if not self.replan_requested:
            return

        self.replan_requested = False  # Reset trigger
        self.get_logger().info("Starting RRT path planning...")

        # Combine all obstacles into one list
        obstacles = self.spherical_obstacles[:]
        obstacles.extend(self.cylindrical_obstacles)
        if self.cylinder_obstacle:
            obstacles.append(self.cylinder_obstacle)

        self.get_logger().debug(f"Obstacles: {obstacles}")
        self.get_logger().debug(f"Starting position: {self.starting_position}")
        self.get_logger().debug(f"Target position: {self.target_position}")

        if self.is_point_in_obstacle(self.starting_position, obstacles) or self.is_point_in_obstacle(self.target_position, obstacles):
            self.get_logger().error("Start or target position is inside an obstacle. Cannot plan path.")
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
            self.get_logger().info(f"Path found in {duration:.2f} seconds. Path: {path}")
            self.publish_path(path)
        else:
            self.get_logger().error("Failed to find a path within the maximum iterations.")

    def distance(self, p1, p2):
        return math.sqrt(sum((p2[i] - p1[i])**2 for i in range(3)))

    def is_point_in_obstacle(self, point, obstacles):
        for obstacle in obstacles:
            if len(obstacle) == 4:  # Spherical obstacle
                ox, oy, oz, r = obstacle
                # Add buffer to radius
                buffered_radius = r + self.obstacle_buffer
                if (point[0] - ox)**2 + (point[1] - oy)**2 + (point[2] - oz)**2 <= buffered_radius**2:
                    return True
            elif len(obstacle) == 5:  # Cylindrical obstacle
                cx, cy, cz, radius, height = obstacle
                dx, dy = point[0] - cx, point[1] - cy
                # Add buffer to radius and height
                buffered_radius = radius + self.obstacle_buffer
                buffered_height = height + (2 * self.obstacle_buffer)  # Add buffer to both top and bottom
                buffered_z_min = cz - self.obstacle_buffer
                if dx**2 + dy**2 <= buffered_radius**2 and buffered_z_min <= point[2] <= buffered_z_min + buffered_height:
                    return True
        return False

    def is_collision_free(self, point1, point2, obstacles):
        """Check if the line segment between point1 and point2 is collision-free."""
        for obstacle in obstacles:
            if len(obstacle) == 4:  # Spherical obstacle
                ox, oy, oz, r = obstacle
                if self.line_intersects_sphere(point1, point2, (ox, oy, oz), r):
                    return False
            elif len(obstacle) == 5:  # Cylindrical obstacle
                cx, cy, cz, radius, height = obstacle
                if self.line_intersects_cylinder(point1, point2, (cx, cy, cz), radius, height):
                    return False
        return True

    def line_intersects_sphere(self, p1, p2, center, radius):
        """Check if the line segment between p1 and p2 intersects a sphere."""
        cx, cy, cz = center
        px, py, pz = p1
        qx, qy, qz = p2
        dx, dy, dz = qx - px, qy - py, qz - pz
        a = dx**2 + dy**2 + dz**2
        b = 2 * (dx * (px - cx) + dy * (py - cy) + dz * (pz - cz))
        # Add buffer to radius
        buffered_radius = radius + self.obstacle_buffer
        c = (px - cx)**2 + (py - cy)**2 + (pz - cz)**2 - buffered_radius**2
        discriminant = b**2 - 4 * a * c
        return discriminant >= 0

    def line_intersects_cylinder(self, p1, p2, center, radius, height):
        """Check if the line segment between p1 and p2 intersects a cylinder."""
        cx, cy, cz = center
        px, py, pz = p1
        qx, qy, qz = p2
        dx, dy = qx - px, qy - py
        a = dx**2 + dy**2
        b = 2 * (dx * (px - cx) + dy * (py - cy))
        # Add buffer to radius
        buffered_radius = radius + self.obstacle_buffer
        c = (px - cx)**2 + (py - cy)**2 - buffered_radius**2
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            return False
        # Add buffer to height (both top and bottom)
        z_min = cz - self.obstacle_buffer
        z_max = cz + height + self.obstacle_buffer
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)
        z1 = pz + t1 * (qz - pz)
        z2 = pz + t2 * (qz - pz)
        return (0 <= t1 <= 1 and z_min <= z1 <= z_max) or (0 <= t2 <= 1 and z_min <= z2 <= z_max)

    def build_rrt(self, start, goal, obstacles, radius, goal_sample_rate, step_size=0.1, max_iters=5000):
        start_time = time.perf_counter()
        start_node = RRTTreeNode(start)
        nodes = [start_node]

        for i in range(max_iters):
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
                    self.get_logger().debug(f"RRT path successfully built in {end_time - start_time:.2f} seconds.")
                    return self.smooth_path(path, obstacles), end_time - start_time

        end_time = time.perf_counter()
        self.get_logger().debug(f"RRT failed to find a path in {end_time - start_time:.2f} seconds.")
        return None, end_time - start_time

    def publish_path(self, path):
        path_msg = Float32MultiArray()
        path_msg.data = [coord for point in path for coord in point]
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published RRT path.")

    def generate_random_point(self, radius):
        """Generate a random point within the workspace radius."""
        while True:
            x = random.uniform(-radius, radius)
            y = random.uniform(-radius, radius)
            z = random.uniform(0, radius)
            if x**2 + y**2 + z**2 <= radius**2:  # Ensure the point is within the sphere
                return [x, y, z]

    def get_nearest_node(self, nodes, point):
        """Find the nearest node in the RRT tree to the given point."""
        nearest_node = None
        min_distance = float('inf')
        for node in nodes:
            dist = self.distance(node.point, point)
            if dist < min_distance:
                nearest_node = node
                min_distance = dist
        return nearest_node

    def extract_path(self, goal_node):
        """Extract the path from the goal node to the start node."""
        path = []
        current = goal_node
        while current is not None:
            path.append(current.point)
            current = current.parent
        path.reverse()  # Reverse the path to go from start to goal
        return path

    def smooth_path(self, path, obstacles):
        """Smooth the path by removing unnecessary intermediate points."""
        if len(path) <= 2:
            return path  # No smoothing needed for paths with 2 or fewer points

        smoothed_path = [path[0]]  # Start with the first point
        current_index = 0

        while current_index < len(path) - 1:
            next_index = len(path) - 1  # Start checking from the last point
            while next_index > current_index + 1:
                if self.is_collision_free(path[current_index], path[next_index], obstacles):
                    break  # Found a valid shortcut
                next_index -= 1
            smoothed_path.append(path[next_index])
            current_index = next_index

        return smoothed_path

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
