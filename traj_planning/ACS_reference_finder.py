#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import numpy as np

class PathMatcher(Node):

    def __init__(self):
        super().__init__('path_matcher_node')

        # Subscribers
        self.pred_pose_sub = self.create_subscription(
            PoseStamped,
            '/predicted_pose',
            self.predicted_pose_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/rrt_trajectory',
            self.path_callback,
            10
        )

        # Publisher for closest point (for debug)
        self.closest_pub = self.create_publisher(
            PoseStamped,
            '/ACS_reference_point',
            10
        )

        self.path_points = []  # Interpolated 100 points

    def path_callback(self, msg: Path):
        poses = msg.poses

        if len(poses) < 2:
            self.get_logger().warn("RRT path has too few points.")
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
            self.get_logger().warn("RRT path total length is zero.")
            return

        # Resample to 100 points
        interp_points = []
        target_dists = np.linspace(0, total_dist, 100)
        for t in target_dists:
            idx = np.searchsorted(cum_dist, t) - 1
            idx = np.clip(idx, 0, len(path_xyz) - 2)

            t1, t2 = cum_dist[idx], cum_dist[idx+1]
            p1, p2 = path_xyz[idx], path_xyz[idx+1]
            alpha = (t - t1) / (t2 - t1 + 1e-8)
            interp = (1 - alpha) * p1 + alpha * p2
            interp_points.append(interp)

        self.path_points = interp_points
        self.get_logger().info("Path interpolated to 100 points.")

    def predicted_pose_callback(self, msg: PoseStamped):
        if not self.path_points:
            self.get_logger().warn("No interpolated path available yet.")
            return

        pred_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Compute distances to all interpolated points
        distances = np.linalg.norm(np.array(self.path_points) - pred_pos, axis=1)
        closest_idx = np.argmin(distances)
        closest_point = self.path_points[closest_idx+10]

        # Publish closest point
        closest_msg = PoseStamped()
        closest_msg.header = msg.header
        closest_msg.pose.position.x = closest_point[0]
        closest_msg.pose.position.y = closest_point[1]
        closest_msg.pose.position.z = closest_point[2]
        closest_msg.pose.orientation.w = 1.0  # neutral orientation

        self.closest_pub.publish(closest_msg)
        self.get_logger().info(f"Closest point idx: {closest_idx}, dist: {distances[closest_idx]:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = PathMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
