#include <memory>
#include <thread>
#include <chrono>
#include <map>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <condition_variable>

geometry_msgs::msg::Pose pose_a;
geometry_msgs::msg::Pose pose_b;

bool pose_a_received = false;
bool pose_b_received = false;

std::mutex pose_mutex;
std::condition_variable pose_cv;

using moveit::planning_interface::MoveGroupInterface;

bool moveToXYZ(
    MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    double x, double y, double z)
{
    geometry_msgs::msg::Pose target;
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;
    target.orientation.w = 1.0;

    move_group.setPoseTarget(target);

    MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        RCLCPP_INFO(logger, "Movement OK");
        return true;
    }

    RCLCPP_ERROR(logger, "Movement Failed");
    return false;
}

bool rotateJoint(
    MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    size_t joint_index,
    double angle_rad)
{
    auto joints = move_group.getCurrentJointValues();

    if (joints.empty() || joint_index >= joints.size())
    {
        RCLCPP_ERROR(logger, "Invalid joint");
        return false;
    }

    joints[joint_index] += angle_rad;
    move_group.setJointValueTarget(joints);

    MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group.execute(plan);
        RCLCPP_INFO(logger, "Joint rotation OK");
        return true;
    }

    RCLCPP_ERROR(logger, "Joint rotation failed");
    return false;
}

bool moveDownZWithLockedJoints(

    MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    double dz_total,
    const std::vector<size_t> &locked_joints,
    double step = 0.02)
{
    auto current_pose = move_group.getCurrentPose().pose;
    double dz_remaining = dz_total;

    while (std::abs(dz_remaining) > 1e-4)
    {
        double dz_step = (std::abs(dz_remaining) < step) ? dz_remaining : (dz_remaining > 0 ? step : -step);

        geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
        target.position.z += dz_step;

        auto joint_values = move_group.getCurrentJointValues();
        auto joint_names = move_group.getJointNames();

        for (size_t idx : locked_joints)
        {
            if (idx < joint_names.size())
            {
                move_group.setJointValueTarget(joint_names[idx], joint_values[idx]);
            }
        }

        move_group.setPoseTarget(target);
        move_group.setGoalPositionTolerance(0.005);
        move_group.setGoalOrientationTolerance(0.01);

        MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(logger, "Planning failed for a small Z-down step");
            return false;
        }

        move_group.execute(plan);
        dz_remaining -= dz_step;

        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    auto final_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Progressive Z descent completed, final Z: %.3f", final_pose.position.z);
    return true;
}

bool moveXWithLockedJoints(
    MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    double dx_total,
    const std::vector<size_t> &locked_joints,
    double step = 0.02 // 2 cm par étape
)
{
    double dx_remaining = dx_total;

    while (std::abs(dx_remaining) > 1e-4)
    {
        double dx_step = (std::abs(dx_remaining) < step) ? dx_remaining : (dx_remaining > 0 ? step : -step);

        geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
        target.position.x += dx_step;

        auto joint_values = move_group.getCurrentJointValues();
        auto joint_names = move_group.getJointNames();

        for (size_t idx : locked_joints)
        {
            if (idx < joint_names.size())
            {
                move_group.setJointValueTarget(joint_names[idx], joint_values[idx]);
            }
        }

        move_group.setPoseTarget(target);
        move_group.setGoalPositionTolerance(0.005);
        move_group.setGoalOrientationTolerance(0.01);

        MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(logger, "Planning failed for a small X-step");
            return false;
        }

        move_group.execute(plan);
        dx_remaining -= dx_step;
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    auto final_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(logger, "X movement completed, final X: %.3f", final_pose.position.x);
    return true;
}

bool moveYWithLockedJoints(
    MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    double dy_total,
    const std::vector<size_t> &locked_joints,
    double step = 0.02)
{
    double dy_remaining = dy_total;

    while (std::abs(dy_remaining) > 1e-4)
    {
        double dy_step = (std::abs(dy_remaining) < step) ? dy_remaining : (dy_remaining > 0 ? step : -step);

        geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
        target.position.y += dy_step;

        auto joint_values = move_group.getCurrentJointValues();
        auto joint_names = move_group.getJointNames();

        for (size_t idx : locked_joints)
        {
            if (idx < joint_names.size())
            {
                move_group.setJointValueTarget(joint_names[idx], joint_values[idx]);
            }
        }

        move_group.setPoseTarget(target);
        move_group.setGoalPositionTolerance(0.005);
        move_group.setGoalOrientationTolerance(0.01);

        MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(logger, "Planning failed for a small Y-step");
            return false;
        }

        move_group.execute(plan);
        dy_remaining -= dy_step;
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    auto final_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Y movement completed, final Y: %.3f", final_pose.position.y);
    return true;
}

void openGripper(
    MoveGroupInterface &gripper_group,
    rclcpp::Logger logger)
{
    std::map<std::string, double> open_pos;
    open_pos["robotiq_85_left_knuckle_joint"] = 0.0;
    open_pos["robotiq_85_right_knuckle_joint"] = 0.0;

    gripper_group.setJointValueTarget(open_pos);
    gripper_group.move();

    RCLCPP_INFO(logger, "Gripper open");
}

bool closeGripper(
    MoveGroupInterface &gripper_group,
    rclcpp::Logger logger,
    int loop_count = 3)
{

    double old_vel = gripper_group.getMaxVelocityScalingFactor();
    gripper_group.setMaxVelocityScalingFactor(0.1);

    std::map<std::string, double> close_pos;
    close_pos["robotiq_85_left_knuckle_joint"] = 0.1;
    close_pos["robotiq_85_right_knuckle_joint"] = -0.1;

    for (int i = 0; i < loop_count; i++)
    {
        gripper_group.setJointValueTarget(close_pos);
        gripper_group.move();
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    gripper_group.setMaxVelocityScalingFactor(old_vel);

    RCLCPP_INFO(logger, "Gripper closed (loop to maximize force via joints)");
    return true;
}

bool posesAreDifferent(
    const geometry_msgs::msg::Pose &p1,
    const geometry_msgs::msg::Pose &p2,
    double tol = 1e-4)
{
    return std::fabs(p1.position.x - p2.position.x) > tol ||
           std::fabs(p1.position.y - p2.position.y) > tol ||
           std::fabs(p1.position.z - p2.position.z) > tol;
}

void targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    rclcpp::Logger logger)
{
    std::lock_guard<std::mutex> lock(pose_mutex);

    if (!pose_a_received)
    {
        pose_a = msg->pose;
        pose_a_received = true;
        RCLCPP_INFO(logger, "Pose A received");
        return;
    }

    if (!pose_b_received && posesAreDifferent(msg->pose, pose_a))
    {
        pose_b = msg->pose;
        pose_b_received = true;
        RCLCPP_INFO(logger, "Pose B received");

        pose_cv.notify_one();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "move_arm",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .append_parameter_override("use_sim_time", true));
    auto logger = node->get_logger();

    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_target_pose",
        10,
        [logger](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            targetPoseCallback(msg, logger);
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]()
                            { executor.spin(); });

    {
        std::unique_lock<std::mutex> lock(pose_mutex);
        RCLCPP_INFO(logger, "Waiting for two distinct target poses...");
        pose_cv.wait(lock, []()
                     { return pose_a_received && pose_b_received; });
    }

    RCLCPP_INFO(logger, "Both target poses received, starting motion sequence");

    MoveGroupInterface arm(node, "ir_arm");
    MoveGroupInterface gripper(node, "ir_gripper");

    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(5);
    arm.allowReplanning(true);
    arm.setMaxVelocityScalingFactor(0.8);
    arm.setMaxAccelerationScalingFactor(0.8);

    for (int i = 0; i < 50 && arm.getCurrentJointValues().empty(); i++)
        rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (arm.getCurrentJointValues().empty())
    {
        RCLCPP_FATAL(logger, "Joints not received");
        return 1;
    }

    float first_x = pose_b.position.x;
    float first_y = pose_b.position.y;
    float first_z = pose_b.position.z;
    float second_x = pose_a.position.x;
    float second_y = pose_a.position.y;
    float second_z = pose_a.position.z;

    if (pose_b.position.x > 4.5 and pose_b.position.x < 4.7)
    {
        first_x = pose_a.position.x;
        first_y = pose_a.position.y;
        first_z = pose_a.position.z;
        second_x = pose_b.position.x;
        second_y = pose_b.position.y;
        second_z = pose_b.position.z;
    }

    openGripper(gripper, logger);
    rclcpp::sleep_for(std::chrono::seconds(1));

    moveToXYZ(arm, logger, second_x + 0.03, second_y, second_z + 0.40);
    /// moveToXYZ(arm, logger, 4.6, -0.5, 0.8);
    rclcpp::sleep_for(std::chrono::seconds(1));

    rotateJoint(arm, logger, arm.getCurrentJointValues().size() - 1, 1.2);
    rotateJoint(arm, logger, arm.getCurrentJointValues().size() - 2, M_PI);

    rclcpp::sleep_for(std::chrono::seconds(1));

    moveDownZWithLockedJoints(
        arm,
        logger,
        -0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2});

    rclcpp::sleep_for(std::chrono::seconds(1));

    closeGripper(gripper, logger);

    rclcpp::sleep_for(std::chrono::seconds(1));

    moveDownZWithLockedJoints(
        arm,
        logger,
        0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    rclcpp::sleep_for(std::chrono::seconds(1));

    moveYWithLockedJoints(
        arm,
        logger,
        -0.15,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    moveXWithLockedJoints(
        arm,
        logger,
        -0.40,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    moveYWithLockedJoints(
        arm,
        logger,
        -0.35,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    moveXWithLockedJoints(
        arm,
        logger,
        -0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    moveDownZWithLockedJoints(
        arm,
        logger,
        -0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2});

    openGripper(gripper, logger);

    moveDownZWithLockedJoints(
        arm,
        logger,
        0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2});

    moveYWithLockedJoints(
        arm,
        logger,
        -0.105,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    moveXWithLockedJoints(
        arm,
        logger,
        -0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    rclcpp::sleep_for(std::chrono::seconds(1));
    moveDownZWithLockedJoints(
        arm,
        logger,
        -0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2});

    rclcpp::sleep_for(std::chrono::seconds(1));
    closeGripper(gripper, logger);
    rclcpp::sleep_for(std::chrono::seconds(1));
    moveDownZWithLockedJoints(
        arm,
        logger,
        0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2});

    rclcpp::sleep_for(std::chrono::seconds(1));

    moveYWithLockedJoints(
        arm,
        logger,
        0.35,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );
    rclcpp::sleep_for(std::chrono::seconds(1));

    moveXWithLockedJoints(
        arm,
        logger,
        0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );
    moveXWithLockedJoints(
        arm,
        logger,
        0.40,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );
    rclcpp::sleep_for(std::chrono::seconds(1));
    moveYWithLockedJoints(
        arm,
        logger,
        0.20,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );
    rclcpp::sleep_for(std::chrono::seconds(1));
    moveDownZWithLockedJoints(
        arm,
        logger,
        0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    openGripper(gripper, logger);

    moveDownZWithLockedJoints(
        arm,
        logger,
        -0.10,
        {arm.getCurrentJointValues().size() - 1,
         arm.getCurrentJointValues().size() - 2}

    );

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}