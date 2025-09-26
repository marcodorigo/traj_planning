import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, WrenchStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from nav_msgs.msg import Path
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

        # Publishers for connecting line and active obstacle marker
        self.connecting_line_publisher = self.create_publisher(Path, '/connecting_line', 10)
        self.active_obstacle_marker_publisher = self.create_publisher(Marker, '/active_obstacle_marker', 10)

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

    def cylinder_obstacle_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) == 5:  # Cylinder has 5 values: x, y, z (center), radius, height
            self.cylinder_obstacle = (data[0], data[1], data[2], data[3], data[4])
        self.get_logger().debug(f"Updated cylinder obstacle: {self.cylinder_obstacle}")

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

        if distance_to_reference > self.distance_threshold:
            self.get_logger().info(f"Distance to ACS reference point exceeded threshold. Triggering replanning.")
            self.replan_requested = True

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > 2:
            current_button_state = msg.buttons[2] if self.use_wii_controller else msg.buttons[1]
            if current_button_state == 1 and self.last_button_state == 0:
                self.replan_requested = True
                self.get_logger().info("Replan requested via joystick button.")
            self.last_button_state = current_button_state

    # RRT Algorithm and Path Planning
    def plan_path(self):
        if not self.replan_requested:
            return

        self.replan_requested = False
        self.get_logger().info("Starting RRT path planning...")

        obstacles = self.spherical_obstacles[:]
        if self.cylinder_obstacle:
            obstacles.append(self.cylinder_obstacle)

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
                if (point[0] - ox)**2 + (point[1] - oy)**2 + (point[2] - oz)**2 <= r**2:
                    return True
            elif len(obstacle) == 5:  # Cylindrical obstacle
                cx, cy, cz, radius, height = obstacle
                dx, dy = point[0] - cx, point[1] - cy
                if dx**2 + dy**2 <= radius**2 and cz <= point[2] <= cz + height:
                    return True
        return False

    def is_collision_free(self, point1, point2, obstacles):
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
        cx, cy, cz = center
        px, py, pz = p1
        qx, qy, qz = p2
        dx, dy, dz = qx - px, qy - py, qz - pz
        a = dx**2 + dy**2 + dz**2
        b = 2 * (dx * (px - cx) + dy * (py - cy) + dz * (pz - cz))
        c = (px - cx)**2 + (py - cy)**2 + (pz - cz)**2 - radius**2
        discriminant = b**2 - 4 * a * c
        return discriminant >= 0

    def line_intersects_cylinder(self, p1, p2, center, radius, height):
        cx, cy, cz = center
        px, py, pz = p1
        qx, qy, qz = p2
        dx, dy = qx - px, qy - py
        a = dx**2 + dy**2
        b = 2 * (dx * (px - cx) + dy * (py - cy))
        c = (px - cx)**2 + (py - cy)**2 - radius**2
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            return False
        z_min, z_max = cz, cz + height
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)
        z1 = pz + t1 * (qz - pz)
        z2 = pz + t2 * (qz - pz)
        return (0 <= t1 <= 1 and z_min <= z1 <= z_max) or (0 <= t2 <= 1 and z_min <= z2 <= z_max)

    def build_rrt(self, start, goal, obstacles, radius, goal_sample_rate, step_size=0.1, max_iters=5000):
        start_time = time.perf_counter()
        start_node = RRTTreeNode(start)
        nodes = [start_node]

        bias_distance = 0.2
        bias_radius = 0.3
        p_bias = 0.3

        for i in range(max_iters):
            connecting_line = (start, goal)
            self.publish_connecting_line(connecting_line)

            intersecting_obstacles = self.find_intersecting_obstacles(connecting_line, obstacles)

            if intersecting_obstacles:
                active_obstacle = self.get_closest_obstacle(intersecting_obstacles, start)
                obs_center = active_obstacle[:3]
                self.publish_active_obstacle_marker(obs_center)

                offset_dir = self.normalize(self.ho_force_direction())
                via_point = [obs_center[j] + bias_distance * offset_dir[j] for j in range(3)]
            else:
                via_point = None

            if via_point and random.random() < p_bias:
                sample = self.sample_near(via_point, bias_radius)
            else:
                sample = self.generate_random_point(radius)

            nearest = self.get_nearest_node(nodes, sample)
            direction = [sample[j] - nearest.point[j] for j in range(3)]
            norm = math.sqrt(sum(d**2 for d in direction))
            if norm == 0:
                continue
            direction = [d / norm for d in direction]
            new_point = [nearest.point[j] + step_size * direction[j] for j in range(3)]

            if self.is_collision_free(nearest.point, new_point, obstacles):
                new_node = RRTTreeNode(new_point, nearest)
                nodes.append(new_node)

                if self.distance(new_point, goal) <= step_size and self.is_collision_free(new_point, goal, obstacles):
                    goal_node = RRTTreeNode(goal, new_node)
                    nodes.append(goal_node)
                    path = self.extract_path(goal_node)
                    end_time = time.perf_counter()
                    self.get_logger().debug(f"Biased RRT path successfully built in {end_time - start_time:.2f} seconds.")
                    return self.smooth_path(path, obstacles), end_time - start_time

        end_time = time.perf_counter()
        self.get_logger().debug(f"Biased RRT failed to find a path in {end_time - start_time:.2f} seconds.")
        return None, end_time - start_time

    def find_intersecting_obstacles(self, line, obstacles):
        """
        Find obstacles that intersect the given line segment.
        :param line: A tuple containing the start and end points of the line segment (start, end).
        :param obstacles: A list of obstacles, where each obstacle is either a sphere (x, y, z, radius)
                          or a cylinder (x, y, z, radius, height).
        :return: A list of obstacles that intersect the line segment.
        """
        start, end = line
        intersecting_obstacles = []

        for obstacle in obstacles:
            if len(obstacle) == 4:  # Spherical obstacle
                center = obstacle[:3]
                radius = obstacle[3]
                if self.line_intersects_sphere(start, end, center, radius):
                    intersecting_obstacles.append(obstacle)
            elif len(obstacle) == 5:  # Cylindrical obstacle
                center = obstacle[:3]
                radius = obstacle[3]
                height = obstacle[4]
                if self.line_intersects_cylinder(start, end, center, radius, height):
                    intersecting_obstacles.append(obstacle)

        return intersecting_obstacles

    def publish_connecting_line(self, connecting_line):
        start, goal = connecting_line
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        start_pose = PoseStamped()
        start_pose.header = path_msg.header
        start_pose.pose.position.x = start[0]
        start_pose.pose.position.y = start[1]
        start_pose.pose.position.z = start[2]
        path_msg.poses.append(start_pose)

        goal_pose = PoseStamped()
        goal_pose.header = path_msg.header
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = goal[2]
        path_msg.poses.append(goal_pose)

        self.connecting_line_publisher.publish(path_msg)
        self.get_logger().debug("Published connecting line.")

    def publish_active_obstacle_marker(self, obs_center):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "active_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = obs_center[0]
        marker.pose.position.y = obs_center[1]
        marker.pose.position.z = obs_center[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.active_obstacle_marker_publisher.publish(marker)
        self.get_logger().debug("Published active obstacle marker.")

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
