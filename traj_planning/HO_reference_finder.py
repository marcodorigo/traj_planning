#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from std_msgs.msg import Header
import numpy as np
import copy
import time

class AdmittancePredictor(Node):

    def __init__(self):
        super().__init__('admittance_predictor_node')

        # Constants
        self.m = 10.0
        self.c = 100.0
        self.k = 0.0  # not used here

        # Initial states
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.force = np.zeros(3)
        self.last_time = None

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/admittance_controller/pose_debug',
            self.pose_callback,
            10
        )

        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/ho_wrench_topic',
            self.force_callback,
            10
        )

        # Publisher for predicted pose
        self.pred_pose_pub = self.create_publisher(
            PoseStamped,
            '/predicted_pose',
            10
        )

        # Timer for prediction update
        self.timer = self.create_timer(0.01, self.predict_pose)  # 100 Hz

    def pose_callback(self, msg: PoseStamped):
        self.position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.last_time = self.get_clock().now()

    def force_callback(self, msg: WrenchStamped):
        self.force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

    def predict_pose(self):
        if self.last_time is None:
            return

        # Time step
        current_time = self.get_clock().now()
        #dt = (current_time - self.last_time).nanoseconds * 1e-9
        dt = 10  # Fixed time step for simplicity and debugging
        if dt <= 0.0:
            return

        # Admittance dynamics: a = (F - c*v)/m
        acceleration = (self.force - self.c * self.velocity) / self.m

        # Euler integration
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # Create PoseStamped message
        pred_msg = PoseStamped()
        pred_msg.header = Header()
        pred_msg.header.stamp = self.get_clock().now().to_msg()
        pred_msg.header.frame_id = "base_link"

        pred_msg.pose.position.x = self.position[0]
        pred_msg.pose.position.y = self.position[1]
        pred_msg.pose.position.z = self.position[2]

        # Orientation unchanged (use identity quaternion)
        pred_msg.pose.orientation.w = 1.0
        pred_msg.pose.orientation.x = 0.0
        pred_msg.pose.orientation.y = 0.0
        pred_msg.pose.orientation.z = 0.0

        # Publish
        self.pred_pose_pub.publish(pred_msg)
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = AdmittancePredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
