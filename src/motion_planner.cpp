#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <string>

// ─────────────────────────────────────────────
// 1. STRUCT DEFINITION — goes at top, after includes, before main()
// ─────────────────────────────────────────────
struct Waypoint {
    std::string name;
    geometry_msgs::msg::Pose pose;
    enum Action { NONE, OPEN_GRIPPER, CLOSE_GRIPPER } action_after;
};

// ─────────────────────────────────────────────
// 2. HELPER FUNCTION — also before main(), after the struct
// ─────────────────────────────────────────────
bool move_to_waypoint(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const Waypoint& wp,
    rclcpp::Logger logger)
{
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(wp.pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(logger, "Executing waypoint: %s", wp.name.c_str());
        move_group.execute(plan);
        return true;
    } else {
        RCLCPP_ERROR(logger, "Failed to plan waypoint: %s", wp.name.c_str());
        return false;
    }
}

// helper to build a pose quickly — keeps main() readable
geometry_msgs::msg::Pose make_pose(double x, double y, double z) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 0.0;
    p.orientation.x = 1.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    return p;
}

// ─────────────────────────────────────────────
// 3. MAIN — your existing setup code stays the same up top
// ─────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motion_planner",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto logger = node->get_logger();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);

    // gripper publisher + open/close lambdas — same as before
    auto gripper_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/gripper_trajectory_controller/joint_trajectory", 10);

    auto open_gripper = [&]() {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.04, 0.04};
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        msg.points.push_back(point);
        gripper_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(2));
    };

    auto close_gripper = [&]() {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        msg.points.push_back(point);
        gripper_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(2));
    };

    // ─────────────────────────────────────────
    // 4. BUILD THE WAYPOINT LIST — your actual values go here
    // ─────────────────────────────────────────
    std::vector<Waypoint> waypoints = {
        {"pre_grasp", make_pose(0.5245, -0.001, 0.025 + 0.15), Waypoint::NONE},
        {"grasp",     make_pose(0.5245, -0.001, 0.025),        Waypoint::CLOSE_GRIPPER},
        {"retreat",   make_pose(0.5245, -0.001, 0.025 + 0.20), Waypoint::NONE},
        {"drop",      make_pose(0.3, 0.4, 0.5),                Waypoint::OPEN_GRIPPER},
    };

    // ─────────────────────────────────────────
    // 5. THE LOOP — replaces all your repeated blocks
    // ─────────────────────────────────────────
    open_gripper();  // start with gripper open before first move

    for (const auto& wp : waypoints) {
        if (!move_to_waypoint(move_group, wp, logger)) {
            RCLCPP_ERROR(logger, "Stopping sequence due to failure at: %s", wp.name.c_str());
            break;
        }
        if (wp.action_after == Waypoint::OPEN_GRIPPER) open_gripper();
        if (wp.action_after == Waypoint::CLOSE_GRIPPER) close_gripper();
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}