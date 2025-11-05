#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <arm_interface/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = arm_interface::msg::PoseCommand;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));
        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 100, std::bind(&Commander::poseCmdCallback, this, _1));
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();    
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, 
                    double qx, double qy, double qz, double qw,
                    bool cartesian_path=false)
    {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = qx;
        target_pose.pose.orientation.y = qy;
        target_pose.pose.orientation.z = qz;
        target_pose.pose.orientation.w = qw;

        arm_->setStartStateToCurrentState();
        if (!cartesian_path) {
            arm_->setPoseTarget(target_pose);
            MoveGroupInterface::Plan plan;
            bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                arm_->execute(plan);
            }
            else {
                RCLCPP_WARN(node_->get_logger(), 
                        "Initial pose target failed. Setting all joints to 0 and retrying...");
                
                // Set all joints to 0
                std::vector<double> zero_joints(5, 0.0);  // Assuming 5 joints
                arm_->setStartStateToCurrentState();
                arm_->setJointValueTarget(zero_joints);
                
                MoveGroupInterface::Plan zero_plan;
                bool zero_success = (arm_->plan(zero_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (zero_success) {
                    arm_->execute(zero_plan);
                    
                    // Now retry the original pose target
                    RCLCPP_INFO(node_->get_logger(), "Retrying original pose target...");
                    arm_->setStartStateToCurrentState();
                    arm_->setPoseTarget(target_pose);
                    
                    MoveGroupInterface::Plan retry_plan;
                    bool retry_success = (arm_->plan(retry_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    
                    if (retry_success) {
                        arm_->execute(retry_plan);
                        RCLCPP_INFO(node_->get_logger(), "Retry successful!");
                    }
                    else {
                        RCLCPP_ERROR(node_->get_logger(), "Retry failed after resetting to zero position.");
                    }
                }
                else {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to move to zero position.");
                }
            }
        }
        else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            if (fraction == 1) {
                arm_->execute(trajectory);
            }
            else {
                RCLCPP_WARN(node_->get_logger(), 
                        "Cartesian path planning failed. Fraction: %.2f", fraction);
            }
        }
    }

private:
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            interface->execute(plan);
        }
    }

    void jointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;
        if (joints.size() == 5) {
            goToJointTarget(joints);
        }
    }

    void poseCmdCallback(const PoseCmd &msg)
    {
        goToPoseTarget(msg.x, msg.y, msg.z, 
                       msg.ox, msg.oy, msg.oz, msg.ow, 
                       msg.cartesian_path);   
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}