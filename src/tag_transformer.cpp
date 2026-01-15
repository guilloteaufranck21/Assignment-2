#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TagTransformer : public rclcpp::Node {
public:
    TagTransformer() : Node("tag_transformer") {
        //initialization of tf2 buffer(memory) and listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        //from apriltag detector node... from cam frame to world
        subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/detected_tags", 10, std::bind(&TagTransformer::camera_to_world, this, std::placeholders::_1));

        //publishing
        publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_target_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Tag Transformer Node Started. Listening to /detected_tags...");
    }

private:
    void camera_to_world(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try {
        geometry_msgs::msg::PoseStamped transformed_pose;
        
        transformed_pose = tf_buffer->transform(*msg, "world");

        //by this we can get the coordinates of the centre of cube for the gripper to hold it (-0.05 from 0.1 m length og the cube)
        transformed_pose.pose.position.z -= 0.05;

        publisher->publish(transformed_pose);

        RCLCPP_INFO(this->get_logger(), "Transformed to WORLD: X:%.2f Y:%.2f Z:%.2f",
                    transformed_pose.pose.position.x,
                    transformed_pose.pose.position.y,
                    transformed_pose.pose.position.z);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform to world frame: %s", ex.what());
    }
}

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTransformer>());
    rclcpp::shutdown();
    return 0;
}