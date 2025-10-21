#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import numpy as np

class PathMatcher(Node):

    def __init__(self):
        super().__init__('path_matcher_node')

        self.publish_rate = 500.0  # Hz
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_closest_point)
        self.latest_actual_pose = None
        self.latest_reference_point = None  # Store the latest reference point

        # Subscribers
        self.actual_pose_sub = self.create_subscription(PoseStamped, '/admittance_controller/pose_debug', self.actual_pose_callback, 10)
        self.path_sub = self.create_subscription(Path, '/rrt_path_viz', self.path_callback, 10)

        # Publishers
        self.closest_pub = self.create_publisher(PoseStamped, '/ACS_reference_point', 10)

        self.path_points = []  # Interpolated points
        self.circle_radius = 0.05  # Radius for the circle around actual pose

    def path_callback(self, msg: Path):
        poses = msg.poses

        if len(poses) < 2:
            # self.get_logger().warn("RRT path has too few points.")
            return

        # Extract positions
        path_xyz = np.array([
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in poses
        ])

        # Compute cumulative distances along path
        dists = np.linalg.norm(np.diff(path_xyz, axis=0), axis=1)
        cum_dist = np.insert(np.cumsum(dists), 0, 0)
        total_dist = cum_dist[-1]

        if total_dist == 0:
            # self.get_logger().warn("RRT path total length is zero.")
            return

        # Resample to 1000 points
        interp_points = []
        target_dists = np.linspace(0, total_dist, 1000)
        for t in target_dists:
            idx = np.searchsorted(cum_dist, t) - 1
            idx = np.clip(idx, 0, len(path_xyz) - 2)

            t1, t2 = cum_dist[idx], cum_dist[idx+1]
            p1, p2 = path_xyz[idx], path_xyz[idx+1]
            alpha = (t - t1) / (t2 - t1 + 1e-8)
            interp = (1 - alpha) * p1 + alpha * p2
            interp_points.append(interp)

        self.path_points = interp_points
        # self.get_logger().info("Path interpolated to 1000 points.")

    def actual_pose_callback(self, msg: PoseStamped):
        self.latest_actual_pose = msg
        
        if not self.path_points:
            # self.get_logger().warn("No interpolated path available yet.")
            return

        actual_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Find all points within the circle radius
        path_array = np.array(self.path_points)
        distances = np.linalg.norm(path_array - actual_pos, axis=1)
        within_circle_mask = distances <= self.circle_radius
        
        if not np.any(within_circle_mask):
            # If no points within circle, use the closest point
            closest_idx = np.argmin(distances)
            reference_point = self.path_points[closest_idx]
        else:
            # Find the furthest forward point (highest index) within the circle
            within_circle_indices = np.where(within_circle_mask)[0]
            furthest_forward_idx = np.max(within_circle_indices)  # Highest index = furthest forward
            reference_point = self.path_points[furthest_forward_idx]

        # Store the latest reference point
        self.latest_reference_point = PoseStamped()
        self.latest_reference_point.header = msg.header
        self.latest_reference_point.pose.position.x = reference_point[0]
        self.latest_reference_point.pose.position.y = reference_point[1]
        self.latest_reference_point.pose.position.z = reference_point[2]
        self.latest_reference_point.pose.orientation.w = 1.0  # neutral orientation

    def publish_closest_point(self):
        # Publish the latest reference point at a fixed rate
        if self.latest_reference_point is not None:
            self.closest_pub.publish(self.latest_reference_point)

def main(args=None):
    rclpy.init(args=args)
    node = PathMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
