#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <cm/msg/ca_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <thread>
#include <iostream>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using CaCmd = cm::msg::CaCommand;
using namespace std::placeholders;

class Commander2
{
public:
    Commander2(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // ✅ 忽略 joint_states 时间戳，主动等待状态更新
        RCLCPP_INFO(node_->get_logger(), "Waiting for robot state (ignore stamp=0)...");
        bool valid = false;
        for (int i = 0; i < 20; ++i)
        {
            auto state = arm_->getCurrentState();
            if (state && state->getVariableCount() > 0)
            {
                const double *positions = state->getVariablePositions();
                if (positions != nullptr)
                {
                    valid = true;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (valid)
            RCLCPP_INFO(node_->get_logger(), "Robot state received (even if stamp=0).");
        else
            RCLCPP_WARN(node_->get_logger(), "No robot state after 10s, continuing anyway.");

        cartesian_cmd_sub_ = node_->create_subscription<CaCmd>(
            "cartesian_command", 10, std::bind(&Commander2::cartesianCmdCallback, this, _1));

        RCLCPP_INFO(node_->get_logger(), "Commander2 initialized. Waiting for Cartesian commands...");
    }

    void executeCartesianMotion(double dx, double dy, double dz)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose current_pose = arm_->getCurrentPose().pose;

        std::cout << "Current Position: ["
                  << current_pose.position.x << ", "
                  << current_pose.position.y << ", "
                  << current_pose.position.z << "]" << std::endl;

        std::cout << "Current Orientation: ["
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << ", "
                  << current_pose.orientation.w << "]" << std::endl;

        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;

        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

        if (fraction > 0.0) {
            arm_->execute(trajectory);
            RCLCPP_INFO(node_->get_logger(), "Cartesian path executed successfully (fraction=%.2f).", fraction);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "Cartesian path planning incomplete. Fraction: %.2f", fraction);
        }
    }

private:
    void cartesianCmdCallback(const CaCmd &msg)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received Cartesian command: dx=%.3f, dy=%.3f, dz=%.3f",
                    msg.dx, msg.dy, msg.dz);
        executeCartesianMotion(msg.dx, msg.dy, msg.dz);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::Subscription<CaCmd>::SharedPtr cartesian_cmd_sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander2");
    Commander2 commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
