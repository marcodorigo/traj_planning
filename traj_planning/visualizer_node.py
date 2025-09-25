import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path  # Import Path message


class LineVisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        # Initialize positions and parameters
        self.spherical_obstacles = []  # List of spherical obstacles
        self.cylinder_base = {"center": [0.0, 0.0, 0.0], "radius": 0.0, "height": 0.0}
        self.target_position = [0.0, 0.0, 0.0]  # Default target position

        # Distances, safety coefficient, and decision
        self.dist_to_obstacles = 0.0
        self.dist_to_workspace = 0.0
        self.dist_to_target = 0.0
        self.safety_coefficient = 0.0
        self.selected_game = "Unknown"  # Default value for the selected game

        # Manipulability metric
        self.manipulability = 0.0

        # Subscribers
        self.create_subscription(Float32MultiArray, '/spherical_obstacles', self.spherical_obstacles_callback, 10)
        self.create_subscription(Float32MultiArray, '/cylinder_base', self.cylinder_base_callback, 10)
        self.create_subscription(Float32MultiArray, '/target_position', self.target_position_callback, 10)
        self.create_subscription(PoseStamped, '/distance_metrics', self.distance_metrics_callback, 10)
        self.create_subscription(Float32, '/safety_coefficient', self.safety_coefficient_callback, 10)
        self.create_subscription(Int32, '/differential_gt/decision', self.decision_callback, 10)
        self.create_subscription(Float32, '/manipulability_metric', self.manipulability_callback, 10)
        self.create_subscription(Float32MultiArray, '/rrt_path', self.rrt_path_callback, 10)  # Subscribe to /rrt_path

        # Publishers
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.text_marker_publisher = self.create_publisher(MarkerArray, '/visualization_text_marker_array', 10)
        self.path_publisher = self.create_publisher(Path, '/rrt_path_viz', 10)  # Publisher for /rrt_path_viz

    def spherical_obstacles_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        self.spherical_obstacles = [
            {"center": data[i:i+3], "radius": data[i+3]}
            for i in range(0, len(data), 4)
        ]
        self.publish_markers()

    def cylinder_base_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        self.cylinder_base = {
            "center": data[:3],
            "radius": data[3],
            "height": data[4]
        }
        self.publish_markers()

    def target_position_callback(self, msg: Float32MultiArray):
        self.target_position = list(msg.data)
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
        if msg.data == 1:
            self.selected_game = "NonCoop"
        elif msg.data == 0:
            self.selected_game = "Coop"
        else:
            self.selected_game = "Unknown"
        self.publish_markers()

    def manipulability_callback(self, msg: Float32):
        self.manipulability = msg.data
        self.publish_markers()

    def rrt_path_callback(self, msg: Float32MultiArray):
        """Callback to handle the RRT path."""
        data = list(msg.data)
        path_points = [data[i:i+3] for i in range(0, len(data), 3)]  # Convert flat list to list of points
        self.publish_path(path_points)

    def publish_path(self, path_points):
        """Publish the RRT path as a Path message."""
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published RRT path as a Path message.")

    def make_sphere_marker(self, id, position, radius, color_rgb, namespace):
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
        marker.scale.x = marker.scale.y = marker.scale.z = radius * 2  # Diameter
        marker.color.r = color_rgb[0]
        marker.color.g = color_rgb[1]
        marker.color.b = color_rgb[2]
        marker.color.a = 0.5
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
        text_marker_array = MarkerArray()

        # Add spherical obstacle markers
        for i, obstacle in enumerate(self.spherical_obstacles):
            # Determine obstacle marker color based on distance to obstacles
            obstacle_color = (1.0, 0.0, 0.0) if self.dist_to_obstacles == 0 else (0.0, 0.0, 1.0)

            marker_array.markers.append(
                self.make_sphere_marker(
                    id=100 + i,
                    position=obstacle["center"],
                    radius=obstacle["radius"],
                    color_rgb=obstacle_color,
                    namespace="spherical_obstacle"
                )
            )

        # Add cylinder marker
        marker_array.markers.append(
            self.make_cylinder_marker(
                id=200,
                position=self.cylinder_base["center"],
                radius=self.cylinder_base["radius"],
                height=self.cylinder_base["height"],
                color_rgb=(0.0, 0.5, 0.0),
                namespace="cylinder_obstacle"
            )
        )

        # Add target marker (red sphere)
        marker_array.markers.append(
            self.make_sphere_marker(
                id=300,
                position=self.target_position,
                radius=0.05,  # Fixed radius for the target marker
                color_rgb=(0.0, 1.0, 0.0),  # Green color
                namespace="target_marker"
            )
        )

        # Add text markers for distances and metrics
        text_marker_array.markers.append(self.make_text_marker(400, f"Dist_to_Obstacles: {self.dist_to_obstacles:.2f}",
                                                               [0.5, -0.5, 1.0], "distance_text"))
        text_marker_array.markers.append(self.make_text_marker(401, f"Dist_to_Workspace: {self.dist_to_workspace:.2f}",
                                                               [0.5, -0.5, 0.9], "distance_text"))
        text_marker_array.markers.append(self.make_text_marker(402, f"Dist_to_Target: {self.dist_to_target:.2f}",
                                                               [0.5, -0.5, 0.8], "distance_text"))
        text_marker_array.markers.append(self.make_text_marker(403, f"Manipulability: {self.manipulability:.2f}",
                                                               [0.5, -0.5, 0.7], "manipulability_text"))
        text_marker_array.markers.append(self.make_text_marker(404, f"Safety_Coefficient: {self.safety_coefficient:.2f}",
                                                               [0.5, -0.5, 0.6], "safety_text"))
        text_marker_array.markers.append(self.make_text_marker(405, f"Selected_Game: {self.selected_game}",
                                                               [0.5, -0.5, 0.5], "game_text"))

        # Publish markers
        self.marker_publisher.publish(marker_array)
        self.text_marker_publisher.publish(text_marker_array)

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


def main(args=None):
    rclpy.init(args=args)
    node = LineVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
