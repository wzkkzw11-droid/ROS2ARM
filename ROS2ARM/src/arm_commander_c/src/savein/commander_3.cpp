#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <arm_interface/msg/pose_command.hpp>
#include <cm/msg/ca_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = arm_interface::msg::PoseCommand;
using CaCmd = cm::msg::CaCommand;
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
        
        cartesian_cmd_sub_ = node_->create_subscription<CaCmd>(
            "cartesian_command", 100, std::bind(&Commander::cartesianCmdCallback, this, _1));

        joint_from_pose_pub_ = node_->create_publisher<FloatArray>(
            "joint_from_pose", 10);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();    
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, 
                        double qx, double qy, double qz, double qw)
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
        arm_->setPoseTarget(target_pose);
        
        MoveGroupInterface::Plan plan;
        bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {

            const auto& traj = plan.trajectory.joint_trajectory;
            const auto& last_point = traj.points.back();

            std::cout << "Planned Joint Values:" << std::endl;
            for (size_t i = 0; i < last_point.positions.size(); ++i) {
                std::cout << "Joint " << i << ": " << last_point.positions[i] << std::endl;
            }

            std::ostringstream cmd;
            cmd << "ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray \"{data:[";
            for (size_t i = 0; i < last_point.positions.size(); ++i) {
                cmd << last_point.positions[i];
                if (i < last_point.positions.size() - 1) cmd << ",";
            }
            cmd << "]}\"";
            std::cout << cmd.str() << std::endl;

            FloatArray msg;
            msg.data = last_point.positions;
            joint_from_pose_pub_->publish(msg);

            arm_->execute(plan);
        }
        else {
            RCLCPP_WARN(node_->get_logger(), 
                    "Initial pose target failed. Setting all joints to 0 and retrying...");
            
            std::vector<double> zero_joints(5, 0.0);
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(zero_joints);
            
            MoveGroupInterface::Plan zero_plan;
            bool zero_success = (arm_->plan(zero_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (zero_success) {
                arm_->execute(zero_plan);
                RCLCPP_INFO(node_->get_logger(), "Retrying original pose target...");
                arm_->setStartStateToCurrentState();
                arm_->setPoseTarget(target_pose);
                
                MoveGroupInterface::Plan retry_plan;
                bool retry_success = (arm_->plan(retry_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (retry_success) {
                    arm_->execute(retry_plan);
                    RCLCPP_INFO(node_->get_logger(), "Retry successful!");
                    geometry_msgs::msg::Pose current_pose = arm_->getCurrentPose().pose;
                    RCLCPP_INFO(node_->get_logger(), 
                    "Current Position: [%.6f, %.6f, %.6f]", 
                    current_pose.position.x, 
                    current_pose.position.y, 
                    current_pose.position.z);
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

    void goToCartesianTarget(double dx, double dy, double dz)
    {
        if (!arm_->getCurrentState(3.0)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot state");
            return;
        }
        
        arm_->setStartStateToCurrentState();
        
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose current_pose = arm_->getCurrentPose().pose;
        
        if (current_pose.position.x == 0.0 && current_pose.position.y == 0.0 && 
            current_pose.position.z == 0.0 && current_pose.orientation.w == 0.0) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid current pose - all zeros.");
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                    "Current Position: [%.6f, %.6f, %.6f]", 
                    current_pose.position.x, 
                    current_pose.position.y, 
                    current_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), 
                    "Delta: [%.6f, %.6f, %.6f]", dx, dy, dz);
        
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;
        
        RCLCPP_INFO(node_->get_logger(), 
                    "Target Position: [%.6f, %.6f, %.6f]", 
                    target_pose.position.x, 
                    target_pose.position.y, 
                    target_pose.position.z);
        
        waypoints.push_back(target_pose);
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
        
        if (fraction == 1.0) {
            arm_->execute(trajectory);
            RCLCPP_INFO(node_->get_logger(), "Cartesian path executed successfully.");
        }
        else {
            RCLCPP_WARN(node_->get_logger(), 
                    "Cartesian path planning incomplete. Fraction: %.2f", fraction);
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
                       msg.ox, msg.oy, msg.oz, msg.ow);   
    }

    void cartesianCmdCallback(const CaCmd &msg)
    {
        goToCartesianTarget(msg.dx, msg.dy, msg.dz);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
    rclcpp::Subscription<CaCmd>::SharedPtr cartesian_cmd_sub_;

    // 新增 publisher
    rclcpp::Publisher<FloatArray>::SharedPtr joint_from_pose_pub_;
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
