import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class LineVisualizerNode(Node):
    def __init__(self):
        super().__init__('line_visualizer_node')

        # Initialize positions
        self.starting_position = [0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, 0.0]
        self.obstacle_center = [0.0, 0.0, 0.0]
        self.obstacle_radius = 0.0

        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/admittance_controller/pose_debug',
            self.starting_position_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_position_callback,
            10
        )
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
            Float32MultiArray,
            '/rrt_path',
            self.path_callback,
            10
        )

        # Publisher for RViz markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )

        self.get_logger().info("✅ Line Visualizer Node Initialized")

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.get_logger().info(f"Updated starting position: {self.starting_position}")
        self.publish_markers()

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]
        self.get_logger().info(f"Updated target position: {self.target_position}")
        self.publish_markers()

    def obstacle_center_callback(self, msg: Float32MultiArray):
        self.obstacle_center = msg.data[:3]
        self.get_logger().info(f"Updated obstacle center: {self.obstacle_center}")
        self.publish_markers()

    def obstacle_radius_callback(self, msg: Float32):
        self.obstacle_radius = msg.data
        self.get_logger().info(f"Updated obstacle radius: {self.obstacle_radius}")
        self.publish_markers()

    def path_callback(self, msg: Float32MultiArray):
        path = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        self.get_logger().info(f"Received path with {len(path)} points.")
        self.publish_path_marker(path)

    def publish_markers(self):
        # Create a MarkerArray
        marker_array = MarkerArray()

        # Start marker
        start_marker = Marker()
        start_marker.header.frame_id = "base_link"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start_marker"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.starting_position[0]
        start_marker.pose.position.y = self.starting_position[1]
        start_marker.pose.position.z = self.starting_position[2]
        start_marker.scale.x = 0.1
        start_marker.scale.y = 0.1
        start_marker.scale.z = 0.1
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0  # Green for start
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        marker_array.markers.append(start_marker)

        # Target marker
        target_marker = Marker()
        target_marker.header.frame_id = "base_link"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target_marker"
        target_marker.id = 1
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = self.target_position[0]
        target_marker.pose.position.y = self.target_position[1]
        target_marker.pose.position.z = self.target_position[2]
        target_marker.scale.x = 0.1
        target_marker.scale.y = 0.1
        target_marker.scale.z = 0.1
        target_marker.color.r = 1.0  # Red for target
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0
        marker_array.markers.append(target_marker)

        # Obstacle marker
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "base_link"
        obstacle_marker.header.stamp = self.get_clock().now().to_msg()
        obstacle_marker.ns = "obstacle_marker"
        obstacle_marker.id = 2
        obstacle_marker.type = Marker.SPHERE
        obstacle_marker.action = Marker.ADD
        obstacle_marker.pose.position.x = self.obstacle_center[0]
        obstacle_marker.pose.position.y = self.obstacle_center[1]
        obstacle_marker.pose.position.z = self.obstacle_center[2]
        obstacle_marker.scale.x = self.obstacle_radius * 2  # Diameter
        obstacle_marker.scale.y = self.obstacle_radius * 2
        obstacle_marker.scale.z = self.obstacle_radius * 2
        obstacle_marker.color.r = 0.0
        obstacle_marker.color.g = 0.0
        obstacle_marker.color.b = 1.0  # Blue for obstacle
        obstacle_marker.color.a = 0.5  # Semi-transparent
        marker_array.markers.append(obstacle_marker)

        # Publish the markers
        self.marker_publisher.publish(marker_array)

    def publish_path_marker(self, path):
        from geometry_msgs.msg import Point

        # Create a Marker for the path
        path_marker = Marker()
        path_marker.header.frame_id = "base_link"  # Replace with your robot's base frame
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path_marker"
        path_marker.id = 3
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05  # Line width
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 1.0  # Blue for path
        path_marker.color.a = 1.0

        # Add points to the path marker
        for point in path:
            path_point = Point()
            path_point.x = point[0]
            path_point.y = point[1]
            path_point.z = point[2]
            path_marker.points.append(path_point)

        # Wrap the path marker in a MarkerArray
        marker_array = MarkerArray()
        marker_array.markers.append(path_marker)

        # Publish the MarkerArray
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LineVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()