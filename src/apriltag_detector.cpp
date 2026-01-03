#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp"

class AprilTagDetector : public rclcpp::Node
{
public:
    AprilTagDetector()
        : Node("apriltag_detector")
    {
        detection_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag/detections", 10, std::bind(&AprilTagDetector::detectionCallback, this, std::placeholders::_1));

        red_cube_pos_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_poses/red", 10);
        blue_cube_pos_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_poses/blue", 10);
    }

    void detectionCallback(
        const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        for (long unsigned int i = 0; i < sizeof(msg->detections); i++)
        {
            apriltag_msgs::msg::AprilTagDetection detection;
            detection = msg->detections[i];
            int tag_id = detection.id;

            geometry_msgs::msg::PoseStamped pose_msg;
            std_msgs::msg::Header pose_header;
            pose_header.frame_id = "base_link";
            pose_header.stamp = this->get_clock()->now();
            pose_msg.header = pose_header;
            pose_msg.pose.position.x = detection.centre.x;
            pose_msg.pose.position.y = detection.centre.y;

            if (tag_id == 1)
            { /* Red Cube */
                red_cube_pos_->publish(pose_msg);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Detected Red Cube");
            }

            else if (tag_id == 10)
            { /* Red Cube */
                blue_cube_pos_->publish(pose_msg);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Detected Blue Cube");
            }
        }
    }

private:
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr red_cube_pos_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr blue_cube_pos_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagDetector>());
    rclcpp::shutdown();
    return 0;
}