import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String, Float32MultiArray
import math
import time
import json

class CollisionAwarePlanner(Node):
    def __init__(self):
        super().__init__('collision_aware_planner')

        # Initialize MoveIt 2 interfaces
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"  # Default MoveGroup name for UR5e
        self.move_group = MoveGroupCommander(self.group_name)

        # Subscriber to get the current end-effector pose
        self.create_subscription(PoseStamped, '/admittance_controller/pose_debug', self.pose_callback, 10)

        # Subscriber to get spherical obstacles
        self.create_subscription(Float32MultiArray, '/spherical_obstacles', self.obstacles_callback, 10)

        # Subscriber to get the target position
        self.create_subscription(Float32MultiArray, '/target_position', self.target_position_callback, 10)

        # Publisher to visualize the planned trajectory in RViz
        self.trajectory_publisher = self.create_publisher(String, '/display_planned_path', 10)

        self.current_pose = None
        self.target_pose = None

    def pose_callback(self, msg):
        """Callback to update the robot's current end-effector pose."""
        self.current_pose = msg
        self.get_logger().info("Updated current pose from /admittance_controller/pose_debug")

    def obstacles_callback(self, msg):
        """Callback to update obstacles in the planning scene."""
        data = msg.data
        if len(data) % 4 != 0:
            self.get_logger().error("Invalid obstacle data received. Each obstacle must have 3 coordinates and 1 radius.")
            return

        for i in range(0, len(data), 4):
            sphere_pose = PoseStamped()
            sphere_pose.header.frame_id = "world"
            sphere_pose.pose.position.x = data[i]
            sphere_pose.pose.position.y = data[i + 1]
            sphere_pose.pose.position.z = data[i + 2]
            sphere_pose.pose.orientation.w = 1.0

            radius = data[i + 3]
            obstacle_id = f"obstacle_{i // 4}"

            self.scene.add_sphere(obstacle_id, sphere_pose, radius=radius)
            self.get_logger().info(f"Added/Updated spherical obstacle: {obstacle_id}")

    def target_position_callback(self, msg):
        """Callback to update the target pose."""
        data = msg.data
        if len(data) != 3:
            self.get_logger().error("Invalid target position data received. Must contain exactly 3 values (x, y, z).")
            return

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "world"
        self.target_pose.pose.position.x = data[0]
        self.target_pose.pose.position.y = data[1]
        self.target_pose.pose.position.z = data[2]
        self.target_pose.pose.orientation.w = 1.0

        self.get_logger().info("Updated target pose from /target_position")

    def plan_to_target(self):
        """Plan a collision-aware trajectory to the target pose."""
        if self.current_pose is None:
            self.get_logger().error("Current pose is not available. Cannot plan.")
            return

        if self.target_pose is None:
            self.get_logger().error("Target pose is not available. Cannot plan.")
            return

        # Set the start state to the robot's current configuration
        self.move_group.set_start_state_to_current_state()

        # Set the target pose
        self.move_group.set_pose_target(self.target_pose.pose)

        # Plan the trajectory
        self.get_logger().info("Planning to target pose...")
        plan = self.move_group.plan()

        if plan[0]:
            self.get_logger().info("Plan found successfully.")

            # Optionally publish the trajectory to RViz
            self.publish_trajectory(plan)
        else:
            self.get_logger().error("Failed to find a plan.")

    def publish_trajectory(self, plan):
        """Publish the planned trajectory to RViz for visualization."""
        trajectory_msg = String()
        trajectory_msg.data = str(plan[1])  # Plan contains a tuple, the second element is the trajectory
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info("Published planned trajectory to /display_planned_path.")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAwarePlanner()

    # Wait for the current pose and target pose to be available
    while rclpy.ok() and (node.current_pose is None or node.target_pose is None):
        rclpy.spin_once(node)
        time.sleep(0.1)

    # Plan to the target pose
    node.plan_to_target()

    rclpy.shutdown()

if __name__ == '__main__':
    main()