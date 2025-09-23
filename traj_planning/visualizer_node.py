import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32  # Update the import to include Int32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path


class LineVisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        # Initialize positions
        self.starting_position = [0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, 0.0]
        self.obstacle_center = [0.0, 0.0, 0.0]
        self.obstacle_radius = 0.0
        self.predicted_position = [0.0, 0.0, 0.0]
        self.closest_path_point = [0.0, 0.0, 0.0]

        # Distances, safety coefficient, and decision
        self.dist_to_obstacles = 0.0
        self.dist_to_workspace = 0.0
        self.dist_to_target = 0.0
        self.safety_coefficient = 0.0
        self.selected_game = "Unknown"  # Default value for the selected game

        # Manipulability metric
        self.manipulability = 0.0

        # Subscribers
        self.create_subscription(PoseStamped, '/admittance_controller/pose_debug', self.starting_position_callback, 10)
        self.create_subscription(Float32MultiArray, '/target_position', self.target_position_callback, 10)
        self.create_subscription(Float32MultiArray, '/obstacle_center', self.obstacle_center_callback, 10)
        self.create_subscription(Float32, '/obstacle_radius', self.obstacle_radius_callback, 10)
        self.create_subscription(Float32MultiArray, '/rrt_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/predicted_pose', self.predicted_pose_callback, 10)
        self.create_subscription(PoseStamped, '/ACS_reference_point', self.closest_point_callback, 10)

        # NEW: Subscribers for distances, safety coefficient, and decision
        self.create_subscription(PoseStamped, '/distance_metrics', self.distance_metrics_callback, 10)
        self.create_subscription(Float32, '/safety_coefficient', self.safety_coefficient_callback, 10)
        self.create_subscription(Int32, '/differential_gt/decision', self.decision_callback, 10)  # Update the subscriber to use Int32

        # NEW: Subscriber for manipulability metric
        self.create_subscription(Float32, '/manipulability_metric', self.manipulability_callback, 10)

        # Publishers
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.text_marker_publisher = self.create_publisher(MarkerArray, '/visualization_text_marker_array', 10)  # New topic
        self.path_publisher = self.create_publisher(Path, '/rrt_trajectory', 10)

        # self.get_logger().info("Line Visualizer Node Initialized")

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.publish_markers()

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]
        self.publish_markers()

    def obstacle_center_callback(self, msg: Float32MultiArray):
        self.obstacle_center = msg.data[:3]
        self.publish_markers()

    def obstacle_radius_callback(self, msg: Float32):
        self.obstacle_radius = msg.data
        self.publish_markers()

    def path_callback(self, msg: Float32MultiArray):
        path = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        self.publish_path_trajectory(path)

    def predicted_pose_callback(self, msg: PoseStamped):
        self.predicted_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.publish_markers()

    def closest_point_callback(self, msg: PoseStamped):
        self.closest_path_point = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.publish_markers()

    def distance_metrics_callback(self, msg: PoseStamped):
        self.dist_to_obstacles = msg.pose.position.x
        self.dist_to_workspace = msg.pose.position.y
        self.dist_to_target = msg.pose.position.z
        self.publish_markers()

    def safety_coefficient_callback(self, msg: Float32):
        self.safety_coefficient = msg.data
        self.publish_markers()

    def decision_callback(self, msg: Int32):
        # Update the selected game based on the decision value
        if msg.data == 1:
            self.selected_game = "NonCoop"
        elif msg.data == 0:
            self.selected_game = "Coop"
        else:
            self.selected_game = "Unknown"
        self.publish_markers()

    # NEW: Callback for manipulability metric
    def manipulability_callback(self, msg: Float32):
        self.manipulability = msg.data
        self.publish_markers()

    def make_sphere_marker(self, id, position, color_rgb, namespace):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = color_rgb[0]
        marker.color.g = color_rgb[1]
        marker.color.b = color_rgb[2]
        marker.color.a = 1.0
        return marker

    def make_text_marker(self, id, text, position, namespace):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.z = 0.1  # Text size
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker

    def make_cylinder_marker(self, id, position, radius, height, color_rgb, namespace):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + height / 2.0  # Center the cylinder vertically
        marker.scale.x = marker.scale.y = radius * 2  # Diameter
        marker.scale.z = height
        marker.color.r = color_rgb[0]
        marker.color.g = color_rgb[1]
        marker.color.b = color_rgb[2]
        marker.color.a = 0.2
        return marker

    def publish_markers(self):
        marker_array = MarkerArray()
        text_marker_array = MarkerArray()  # Separate MarkerArray for text markers
        time_now = self.get_clock().now().to_msg()

        # Add sphere markers
        marker_array.markers.append(self.make_sphere_marker(1, self.target_position, (1.0, 0.0, 0.0), "target_marker"))
        
        # Obstacle marker with conditional color
        obstacle_color = (1.0, 0.0, 0.0) if self.dist_to_obstacles == 0 else (0.0, 0.0, 1.0)
        obstacle_marker = self.make_sphere_marker(2, self.obstacle_center, obstacle_color, "obstacle_marker")
        obstacle_marker.scale.x = obstacle_marker.scale.y = obstacle_marker.scale.z = self.obstacle_radius * 2
        obstacle_marker.color.a = 0.5
        marker_array.markers.append(obstacle_marker)

        # Add cylindrical obstacle marker
        cylinder_position = [0.0, 0.0, 0.0]  # Replace with actual cylinder position
        cylinder_radius = 0.1  # Replace with actual cylinder radius
        cylinder_height = 0.5  # Replace with actual cylinder height
        marker_array.markers.append(self.make_cylinder_marker(3, cylinder_position, cylinder_radius, cylinder_height, (0.0, 1.0, 0.0), "cylinder_marker"))

        # Add text markers to a separate array
        text_marker_array.markers.append(self.make_text_marker(5, f"Dist_to_Obstacles: {self.dist_to_obstacles:.2f}",
                                                              [0.5, -0.5, 1.0], "distance_text"))
        text_marker_array.markers.append(self.make_text_marker(6, f"Dist_to_Workspace: {self.dist_to_workspace:.2f}",
                                                              [0.5, -0.5, 0.9], "distance_text"))
        text_marker_array.markers.append(self.make_text_marker(7, f"Dist_to_Target: {self.dist_to_target:.2f}",
                                                              [0.5, -0.5, 0.8], "distance_text"))
        # NEW: Add manipulability text marker
        text_marker_array.markers.append(self.make_text_marker(8, f"Manipulability: {self.manipulability:.2f}",
                                                              [0.5, -0.5, 0.7], "manipulability_text"))
        text_marker_array.markers.append(self.make_text_marker(9, f"Safety_Coefficient: {self.safety_coefficient:.2f}",
                                                              [0.5, -0.5, 0.6], "safety_text"))
        text_marker_array.markers.append(self.make_text_marker(10, f"Selected_Game: {self.selected_game}",
                                                               [0.5, -0.5, 0.5], "game_text"))  # Updated IDs

        # Publish both marker arrays
        self.marker_publisher.publish(marker_array)
        self.text_marker_publisher.publish(text_marker_array)

    def publish_path_trajectory(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
