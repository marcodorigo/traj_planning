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


# [global_config]
#   window_state = fullscreen
#   suppress_multiple_term_dialog = True
# [keybindings]
# [profiles]
#   [[default]]
#     cursor_color = "#aaaaaa"
# [layouts]
#   [[default]]
#     [[[child0]]]
#       type = Window
#       parent = ""
#       order = 0
#       position = 26:23
#       maximised = False
#       fullscreen = False
#       size = 734, 451
#       title = marco@marco: ~/UR5_ws
#       last_active_term = c2fe6aab-4913-4b67-9417-5998be49c196
#       last_active_window = True
#     [[[terminal1]]]
#       type = Terminal
#       parent = child0
#       order = 0
#       profile = default
#       uuid = c2fe6aab-4913-4b67-9417-5998be49c196
#       command = ""
#   [[rviz]]
#     [[[child0]]]
#       type = Window
#       parent = ""
#       order = 0
#       position = 26:23
#       maximised = False
#       fullscreen = False
#       size = 734, 451
#       title = marco@marco: ~/UR5_ws
#       last_active_term = c2fe6aab-4913-4b67-9417-5998be49c196
#       last_active_window = True
#     [[[child1]]]
#       type = HPaned
#       parent = child0
#       order = 0
#       position = 364
#       ratio = 0.4993141289437586
#     [[[terminal2]]]
#       type = Terminal
#       parent = child1
#       order = 0
#       profile = default
#       uuid = c2fe6aab-4913-4b67-9417-5998be49c196
#       command = '''bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch ur5e_bringup fake_start.launch.py initial_joint_controller:="forward_velocity_controller"; exec bash''''
#     [[[terminal3]]]
#       type = Terminal
#       parent = child1
#       order = 1
#       profile = default
#       uuid = a90026bd-0777-4875-a371-f37c3eaa2e68
#       command = ""
#   [[wii]]
#     [[[child0]]]
#       type = Window
#       parent = ""
#       order = 0
#       position = 26:23
#       maximised = False
#       fullscreen = False
#       size = 734, 451
#       title = marco@marco: ~
#       last_active_term = 6766d1e0-7237-4333-9000-9d989db245a7
#       last_active_window = True
#     [[[child1]]]
#       type = HPaned
#       parent = child0
#       order = 0
#       position = 364
#       ratio = 0.4993141289437586
#     [[[child2]]]
#       type = VPaned
#       parent = child1
#       order = 0
#       position = 223
#       ratio = 0.5
#     [[[terminal3]]]
#       type = Terminal
#       parent = child2
#       order = 0
#       profile = default
#       uuid = 6766d1e0-7237-4333-9000-9d989db245a7
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch compliance_controller admittance_controller.launch.py virtual_wrench_off:=false measured_torque_off:=true use_dummy_force_sensor:=true; exec bash'"
#     [[[child4]]]
#       type = VPaned
#       parent = child2
#       order = 1
#       position = 109
#       ratio = 0.5
#     [[[terminal5]]]
#       type = Terminal
#       parent = child4
#       order = 0
#       profile = default
#       uuid = 461a8ca4-dc6d-4256-b6f2-029563474a32
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch differential_gt differential_gt_sim.launch.py; exec bash'"
#     [[[child6]]]
#       type = VPaned
#       parent = child4
#       order = 1
#       position = 52
#       ratio = 0.5
#     [[[terminal7]]]
#       type = Terminal
#       parent = child6
#       order = 0
#       profile = default
#       uuid = 5daa9541-ce8d-45bb-8be1-c184f3bad8ca
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run differential_gt differential_gt; exec bash'"
#     [[[terminal8]]]
#       type = Terminal
#       parent = child6
#       order = 1
#       profile = default
#       uuid = d26c4143-5da5-440e-82a7-5c995e585cd9
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch traj_planning traj_planning_launch.py use_wii_controller:=true; exec bash'"
#     [[[child9]]]
#       type = VPaned
#       parent = child1
#       order = 1
#       position = 223
#       ratio = 0.5
#     [[[terminal10]]]
#       type = Terminal
#       parent = child9
#       order = 0
#       profile = default
#       uuid = 986d3859-1887-4eb3-b6ac-5a0d74612e92
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch joy_to_wrench joystick_launch.py; exec bash'"
#     [[[child11]]]
#       type = VPaned
#       parent = child9
#       order = 1
#       position = 109
#       ratio = 0.5
#     [[[terminal12]]]
#       type = Terminal
#       parent = child11
#       order = 0
#       profile = default
#       uuid = 5ba378de-1f30-488e-b773-309402a281da
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch fuzzy_inference_system fuzzy_inference_system_launch.py; exec bash'"
#     [[[child13]]]
#       type = VPaned
#       parent = child11
#       order = 1
#       position = 52
#       ratio = 0.5
#     [[[terminal14]]]
#       type = Terminal
#       parent = child13
#       order = 0
#       profile = default
#       uuid = 685de651-9fbe-42c5-a49e-ae2ff1ff8f9d
#       command = '''bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch ur5e_bringup fake_start.launch.py initial_joint_controller:="forward_velocity_controller"; exec bash''''
#     [[[terminal15]]]
#       type = Terminal
#       parent = child13
#       order = 1
#       profile = default
#       uuid = bc9679d1-746e-457e-a632-3fa3098a6f52
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash; exec bash'"
#   [[falcon]]
#     [[[child0]]]
#       type = Window
#       parent = ""
#       order = 0
#       position = 26:23
#       maximised = False
#       fullscreen = False
#       size = 734, 451
#       title = marco@marco: ~
#       last_active_term = 986d3859-1887-4eb3-b6ac-5a0d74612e92
#       last_active_window = True
#     [[[child1]]]
#       type = HPaned
#       parent = child0
#       order = 0
#       position = 364
#       ratio = 0.4993141289437586
#     [[[child2]]]
#       type = VPaned
#       parent = child1
#       order = 0
#       position = 223
#       ratio = 0.5
#     [[[terminal3]]]
#       type = Terminal
#       parent = child2
#       order = 0
#       profile = default
#       uuid = 6766d1e0-7237-4333-9000-9d989db245a7
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch compliance_controller admittance_controller.launch.py virtual_wrench_off:=false measured_torque_off:=true use_dummy_force_sensor:=true; exec bash'"
#     [[[child4]]]
#       type = VPaned
#       parent = child2
#       order = 1
#       position = 109
#       ratio = 0.5
#     [[[terminal5]]]
#       type = Terminal
#       parent = child4
#       order = 0
#       profile = default
#       uuid = 461a8ca4-dc6d-4256-b6f2-029563474a32
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch differential_gt differential_gt_sim.launch.py; exec bash'"
#     [[[child6]]]
#       type = VPaned
#       parent = child4
#       order = 1
#       position = 52
#       ratio = 0.5
#     [[[terminal7]]]
#       type = Terminal
#       parent = child6
#       order = 0
#       profile = default
#       uuid = 5daa9541-ce8d-45bb-8be1-c184f3bad8ca
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run differential_gt differential_gt; exec bash'"
#     [[[terminal8]]]
#       type = Terminal
#       parent = child6
#       order = 1
#       profile = default
#       uuid = d26c4143-5da5-440e-82a7-5c995e585cd9
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch traj_planning traj_planning_launch.py; exec bash'"
#     [[[child9]]]
#       type = VPaned
#       parent = child1
#       order = 1
#       position = 223
#       ratio = 0.5
#     [[[terminal10]]]
#       type = Terminal
#       parent = child9
#       order = 0
#       profile = default
#       uuid = 986d3859-1887-4eb3-b6ac-5a0d74612e92
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run falcon master; exec bash'"
#     [[[child11]]]
#       type = VPaned
#       parent = child9
#       order = 1
#       position = 109
#       ratio = 0.5
#     [[[terminal12]]]
#       type = Terminal
#       parent = child11
#       order = 0
#       profile = default
#       uuid = 5ba378de-1f30-488e-b773-309402a281da
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run falcon falcon_joystick; exec bash'"
#     [[[child13]]]
#       type = VPaned
#       parent = child11
#       order = 1
#       position = 52
#       ratio = 0.5
#     [[[terminal14]]]
#       type = Terminal
#       parent = child13
#       order = 0
#       profile = default
#       uuid = 685de651-9fbe-42c5-a49e-ae2ff1ff8f9d
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch fuzzy_inference_system fuzzy_inference_system_launch.py; exec bash'"
#     [[[terminal15]]]
#       type = Terminal
#       parent = child13
#       order = 1
#       profile = default
#       uuid = bc9679d1-746e-457e-a632-3fa3098a6f52
#       command = '''bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch ur5e_bringup fake_start.launch.py initial_joint_controller:="forward_velocity_controller"; exec bash''''
#   [[real]]
#     [[[child0]]]
#       type = Window
#       parent = ""
#       order = 0
#       position = 0:0
#       maximised = True
#       fullscreen = False
#       size = 1010, 656
#       title = marco@marco: ~
#       last_active_term = 6766d1e0-7237-4333-9000-9d989db245a7
#       last_active_window = False
#     [[[child1]]]
#       type = HPaned
#       parent = child0
#       order = 0
#       position = 502
#       ratio = 0.49950248756218907
#     [[[child2]]]
#       type = VPaned
#       parent = child1
#       order = 0
#       position = 326
#       ratio = 0.500768049155146
#     [[[terminal3]]]
#       type = Terminal
#       parent = child2
#       order = 0
#       profile = default
#       uuid = 6766d1e0-7237-4333-9000-9d989db245a7
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch ur5e_bringup real_start.launch.py initial_joint_controller:=forward_velocity_controller; exec bash'"
#     [[[child4]]]
#       type = VPaned
#       parent = child2
#       order = 1
#       position = 160
#       ratio = 0.5
#     [[[terminal5]]]
#       type = Terminal
#       parent = child4
#       order = 0
#       profile = default
#       uuid = 461a8ca4-dc6d-4256-b6f2-029563474a32
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch compliance_controller admittance_controller.launch.py virtual_wrench_off:=false measured_torque_off:=true use_dummy_force_sensor:=true; exec bash'"
#     [[[child6]]]
#       type = VPaned
#       parent = child4
#       order = 1
#       position = 78
#       ratio = 0.5032258064516129
#     [[[terminal7]]]
#       type = Terminal
#       parent = child6
#       order = 0
#       profile = default
#       uuid = 5daa9541-ce8d-45bb-8be1-c184f3bad8ca
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch differential_gt differential_gt_sim.launch.py; exec bash'"
#     [[[terminal8]]]
#       type = Terminal
#       parent = child6
#       order = 1
#       profile = default
#       uuid = d26c4143-5da5-440e-82a7-5c995e585cd9
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run differential_gt differential_gt; exec bash'"
#     [[[child9]]]
#       type = VPaned
#       parent = child1
#       order = 1
#       position = 326
#       ratio = 0.500768049155146
#     [[[terminal10]]]
#       type = Terminal
#       parent = child9
#       order = 0
#       profile = default
#       uuid = 986d3859-1887-4eb3-b6ac-5a0d74612e92
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch traj_planning traj_planning_launch.py; exec bash'"
#     [[[child11]]]
#       type = VPaned
#       parent = child9
#       order = 1
#       position = 160
#       ratio = 0.5
#     [[[terminal12]]]
#       type = Terminal
#       parent = child11
#       order = 0
#       profile = default
#       uuid = 5ba378de-1f30-488e-b773-309402a281da
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run falcon master; exec bash'"
#     [[[child13]]]
#       type = VPaned
#       parent = child11
#       order = 1
#       position = 78
#       ratio = 0.5032258064516129
#     [[[terminal14]]]
#       type = Terminal
#       parent = child13
#       order = 0
#       profile = default
#       uuid = 685de651-9fbe-42c5-a49e-ae2ff1ff8f9d
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 run falcon falcon_joystick; exec bash'"
#     [[[terminal15]]]
#       type = Terminal
#       parent = child13
#       order = 1
#       profile = default
#       uuid = bc9679d1-746e-457e-a632-3fa3098a6f52
#       command = "bash -c 'cd UR5_ws/ && source install/setup.bash && ros2 launch fuzzy_inference_system fuzzy_inference_system_launch.py; exec bash'"
# [plugins]