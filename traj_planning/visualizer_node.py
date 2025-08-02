import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
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
        # Publisher for RViz trajectory display
        self.path_publisher = self.create_publisher(
            Path,
            '/rrt_trajectory',
            10
        )

        self.get_logger().info("Line Visualizer Node Initialized")

    def starting_position_callback(self, msg: PoseStamped):
        self.starting_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        #self.get_logger().info(f"Updated starting position: {self.starting_position}")
        self.publish_markers()

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = msg.data[:3]
        #self.get_logger().info(f"Updated target position: {self.target_position}")
        self.publish_markers()

    def obstacle_center_callback(self, msg: Float32MultiArray):
        self.obstacle_center = msg.data[:3]
        #self.get_logger().info(f"Updated obstacle center: {self.obstacle_center}")
        self.publish_markers()

    def obstacle_radius_callback(self, msg: Float32):
        self.obstacle_radius = msg.data
        #self.get_logger().info(f"Updated obstacle radius: {self.obstacle_radius}")
        self.publish_markers()

    def path_callback(self, msg: Float32MultiArray):
        path = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        #self.get_logger().info(f"Received path with {len(path)} points.")
        self.publish_path_trajectory(path)

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

    def publish_path_trajectory(self, path):
        # Create a Path message
        path_msg = Path()
        path_msg.header.frame_id = "base_link"  # Replace with your robot's base frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Convert the path points to PoseStamped messages
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0  # Default orientation
            path_msg.poses.append(pose)

        # Publish the Path message
        self.path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()