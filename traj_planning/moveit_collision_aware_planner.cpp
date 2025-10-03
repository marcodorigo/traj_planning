#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <string>
#include <vector>
#include <memory>

class CollisionAwarePlanner : public rclcpp::Node
{
public:
    CollisionAwarePlanner()
        : Node("collision_aware_planner"),
          move_group_(std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator"))
    {
        // Initialize subscribers
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/admittance_controller/pose_debug", 10,
            std::bind(&CollisionAwarePlanner::pose_callback, this, std::placeholders::_1));

        obstacles_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/spherical_obstacles", 10,
            std::bind(&CollisionAwarePlanner::obstacles_callback, this, std::placeholders::_1));

        target_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/target_position", 10,
            std::bind(&CollisionAwarePlanner::target_position_callback, this, std::placeholders::_1));

        // Initialize publisher
        trajectory_publisher_ = this->create_publisher<std_msgs::msg::String>("/display_planned_path", 10);

        RCLCPP_INFO(this->get_logger(), "CollisionAwarePlanner node initialized.");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Updated current pose from /admittance_controller/pose_debug");
    }

    void obstacles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        const auto &data = msg->data;
        if (data.size() % 4 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid obstacle data received. Each obstacle must have 3 coordinates and 1 radius.");
            return;
        }

        for (size_t i = 0; i < data.size(); i += 4)
        {
            geometry_msgs::msg::PoseStamped sphere_pose;
            sphere_pose.header.frame_id = "world";
            sphere_pose.pose.position.x = data[i];
            sphere_pose.pose.position.y = data[i + 1];
            sphere_pose.pose.position.z = data[i + 2];
            sphere_pose.pose.orientation.w = 1.0;

            double radius = data[i + 3];
            std::string obstacle_id = "obstacle_" + std::to_string(i / 4);

            planning_scene_interface_.applyCollisionObject(create_sphere(obstacle_id, sphere_pose, radius));
            RCLCPP_INFO(this->get_logger(), "Added/Updated spherical obstacle: %s", obstacle_id.c_str());
        }
    }

    void target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        const auto &data = msg->data;
        if (data.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid target position data received. Must contain exactly 3 values (x, y, z).");
            return;
        }

        target_pose_.header.frame_id = "world";
        target_pose_.pose.position.x = data[0];
        target_pose_.pose.position.y = data[1];
        target_pose_.pose.position.z = data[2];
        target_pose_.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Updated target pose from /target_position");
    }

    void plan_to_target()
    {
        if (!current_pose_.header.frame_id.empty() && !target_pose_.header.frame_id.empty())
        {
            move_group_->setStartStateToCurrentState();
            move_group_->setPoseTarget(target_pose_.pose);

            RCLCPP_INFO(this->get_logger(), "Planning to target pose...");
            auto success = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success)
            {
                RCLCPP_INFO(this->get_logger(), "Plan found successfully.");
                publish_trajectory();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to find a plan.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Current pose or target pose is not available. Cannot plan.");
        }
    }

    void publish_trajectory()
    {
        std_msgs::msg::String trajectory_msg;
        trajectory_msg.data = "Planned trajectory data here"; // Placeholder for actual trajectory data
        trajectory_publisher_->publish(trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "Published planned trajectory to /display_planned_path.");
    }

    moveit_msgs::msg::CollisionObject create_sphere(const std::string &id, const geometry_msgs::msg::PoseStamped &pose, double radius)
    {
        moveit_msgs::msg::CollisionObject sphere;
        sphere.id = id;
        sphere.header = pose.header;
        sphere.primitives.resize(1);
        sphere.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.primitives[0].dimensions.resize(1);
        sphere.primitives[0].dimensions[0] = radius;
        sphere.primitive_poses.push_back(pose.pose);
        sphere.operation = moveit_msgs::msg::CollisionObject::ADD;
        return sphere;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacles_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_publisher_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped target_pose_;

    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionAwarePlanner>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}