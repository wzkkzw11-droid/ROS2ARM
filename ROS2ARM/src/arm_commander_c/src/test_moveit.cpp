#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    //auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Named goal

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("random valid");
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("ramdom_valid");
    
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success2) {
    //     arm.execute(plan2);
    // }

    // --------------------------------------------------------------------------------

    // Joint Goal

    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0};

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // --------------------------------------------------------------------------------

    //Pose Goal

    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    // Print position
    std::cout << std::fixed << std::setprecision(6);  // Control float precision

    std::cout << "target_pose.pose.position.x = " << pose1.position.x << ";" << std::endl;
    std::cout << "target_pose.pose.position.y = " << pose1.position.y << ";" << std::endl;
    std::cout << "target_pose.pose.position.z = " << pose1.position.z << ";" << std::endl;

    std::cout << "target_pose.pose.orientation.x = " << pose1.orientation.x << ";" << std::endl;
    std::cout << "target_pose.pose.orientation.y = " << pose1.orientation.y << ";" << std::endl;
    std::cout << "target_pose.pose.orientation.z = " << pose1.orientation.z << ";" << std::endl;
    std::cout << "target_pose.pose.orientation.w = " << pose1.orientation.w << ";" << std::endl;
    
    std::cout << "ros2 topic pub -1 /pose_command arm_interface/msg/PoseCommand \"{"
              << "x: " << pose1.position.x << ", "
              << "y: " << pose1.position.y << ", "
              << "z: " << pose1.position.z << ", "
              << "ox: " << pose1.orientation.x << ", "
              << "oy: " << pose1.orientation.y << ", "
              << "oz: " << pose1.orientation.z << ", "
              << "ow: " << pose1.orientation.w << ", "
              << "cartesian_path: false}\"" 
              << std::endl;
    target_pose.header.frame_id = "base_link";

    // target_pose.pose.position.x = -0.098143;
    // target_pose.pose.position.y = 0.134608;
    // target_pose.pose.position.z = 0.452547;
    // target_pose.pose.orientation.x = -0.182256;
    // target_pose.pose.orientation.y = -0.039283;
    // target_pose.pose.orientation.z = 0.570163;
    // target_pose.pose.orientation.w = 0.800096;

    target_pose.pose.position.x = 0.081037;
    target_pose.pose.position.y = -0.131730;
    target_pose.pose.position.z = 0.362934;
    target_pose.pose.orientation.x = -0.065699;
    target_pose.pose.orientation.y = -0.331195;
    target_pose.pose.orientation.z = 0.802366;
    target_pose.pose.orientation.w = -0.492140;


    //Position: [0.39137, -4.21521e-05, 0.22646]
    //Orientation: [0.0172663, 0.706912, 0.0172053, 0.706881]

    // target_pose.pose.position.x = 0.39137;
    // target_pose.pose.position.y = -4.21521e-05;
    // target_pose.pose.position.z = 0.22646;
    // target_pose.pose.orientation.x = 0.0172663;
    // target_pose.pose.orientation.y = 0.706912;
    // target_pose.pose.orientation.z = 0.0172053;
    // target_pose.pose.orientation.w = 0.706881;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        // 从规划结果里拿 joint trajectory
        const auto& traj = plan1.trajectory.joint_trajectory;

        // 取轨迹最后一点，也就是目标关节角
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


        // 打印
        std::cout << cmd.str() << std::endl;

        // 如果你想确认再走，可以加 pause 或交互逻辑
        // std::cin.get(); // 回车继续

        arm.execute(plan1);
    }

    // // Cartesian Path

    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    // // Print position
    // std::cout << "Position: [" 
    //         << pose1.position.x << ", "
    //         << pose1.position.y << ", "
    //         << pose1.position.z << "]" << std::endl;

    // // Print orientation (quaternion)
    // std::cout << "Orientation: [" 
    //         << pose1.orientation.x << ", "
    //         << pose1.orientation.y << ", "
    //         << pose1.orientation.z << ", "
    //         << pose1.orientation.w << "]" << std::endl;
    // pose1.position.z += -0.1;
    // waypoints.push_back(pose1);
    // geometry_msgs::msg::Pose pose2 = pose1;
    // pose2.position.x += -0.1;
    // waypoints.push_back(pose2); 
    // geometry_msgs::msg::Pose pose3 = pose2;
    // pose3.position.y += -0.2;
    // pose3.position.z += 0.2;
    // waypoints.push_back(pose3);

    // moveit_msgs::msg::RobotTrajectory trajectory;

    // double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

    // if (fraction == 1) {
    //     arm.execute(trajectory);
    // }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}