#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CubePoseTransformer : public rclcpp::Node
{
public:
    CubePoseTransformer()
        : Node("cube_pose_transformer"), tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        red_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cube_poses/red",
            10,
            std::bind(&CubePoseTransformer::redPoseCallback, this, std::placeholders::_1));

        blue_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cube_poses/blue",
            10,
            std::bind(&CubePoseTransformer::bluePoseCallback, this, std::placeholders::_1));

        red_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cube_poses_base/red", 10);

        blue_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cube_poses_base/blue", 10);
    }

    void redPoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        transformAndPublish(*msg, red_pub_);
    }

    void bluePoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        transformAndPublish(*msg, blue_pub_);
    }

    void transformAndPublish(
        const geometry_msgs::msg::PoseStamped &input_pose,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher)
    {
        geometry_msgs::msg::PoseStamped transformed_pose;

        tf_buffer_.transform(
            input_pose,
            transformed_pose,
            "base_link",
            tf2::durationFromSec(0.2));

        publisher->publish(transformed_pose);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr red_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr blue_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr red_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr blue_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubePoseTransformer>());
    rclcpp::shutdown();
    return 0;
}