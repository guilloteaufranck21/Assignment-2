#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

//AprilTag Header which were unfortunately not working in python :(
#include <apriltag/apriltag.h>  //these libraries help to recognize black and white patterns
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>

//Coordinate transformation requirements
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;


class AprilTagDetector : public rclcpp::Node {
public:
    AprilTagDetector() : Node("apriltag_detector") {
        // Subscribing only to camera topics [cite: 2]
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image", 10, std::bind(&AprilTagDetector::image_callback, this, std::placeholders::_1));
        
        info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rgb_camera/camera_info", 10, std::bind(&AprilTagDetector::info_callback, this, std::placeholders::_1));

        // Output topic for detection results but in rgb_camera frame
        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_tags", 10);

        // Initialize AprilTag components for the 36h11 family 
        tf = tag36h11_create(); //loads the specific pattern type
        td = apriltag_detector_create();  //detector that didn't work in python
        apriltag_detector_add_family(td, tf);  //Linking the family to detector because we are only going to detect 36h11 family and ignore the rest

        RCLCPP_INFO(this->get_logger(), "AprilTag Detector is now preparing to detect the tag 1 and 10, yayy!!");
    }

    //to cleanup the memory but this happens automatically in python, here nooooo!!!
    ~AprilTagDetector() {
        apriltag_detector_destroy(td); 
        tag36h11_destroy(tf);
    }

private:
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        cam_info = msg; // Storing camera intrinsics for pose estimation as this is the func triggered by subscription
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!cam_info) 
        return;

        cv::Mat gray;
        try {
            gray = cv_bridge::toCvShare(msg, "mono8")->image;
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        image_u8_t im = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };
        zarray_t * detections = apriltag_detector_detect(td, &im); //& because we need only address of the image

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t * det;
            zarray_get(detections, i, &det);

            // Filter for ID 1 and Blue Cube ID 10
            if (det->id == 1 || det->id == 10) {
                publish_pose(det, msg->header);
            }
        }
        apriltag_detections_destroy(detections);
    }

    void publish_pose(apriltag_detection_t * det, std_msgs::msg::Header header) {
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.050; // tag size
        info.fx = cam_info->p[0]; 
        info.fy = cam_info->p[5]; //P is the projection matrix
        info.cx = cam_info->p[2]; 
        info.cy = cam_info->p[6];

        apriltag_pose_t pose; 
        estimate_tag_pose(&info, &pose); //here we got the real pnp pose which returns Rotation matrix R and Translational vector T

        if (pose.R && pose.t) {
            geometry_msgs::msg::PoseStamped msg;
            msg.header = header;
            // frame name
            msg.header.frame_id = "external_camera/link/rgb_camera"; 

            // Extract position directly from matrices 
            msg.pose.position.x = MATD_EL(pose.t, 0, 0);
            msg.pose.position.y = MATD_EL(pose.t, 1, 0);
            msg.pose.position.z = MATD_EL(pose.t, 2, 0);

            // Extract rotation and convert to quaternion (from april tag format to more understandable Ros 2 lang) 
            tf2::Matrix3x3 rot(
                MATD_EL(pose.R, 0, 0), 
                MATD_EL(pose.R, 0, 1), 
                MATD_EL(pose.R, 0, 2),
                MATD_EL(pose.R, 1, 0), 
                MATD_EL(pose.R, 1, 1), 
                MATD_EL(pose.R, 1, 2),
                MATD_EL(pose.R, 2, 0), 
                MATD_EL(pose.R, 2, 1), 
                MATD_EL(pose.R, 2, 2));
            
            tf2::Quaternion q; 
            rot.getRotation(q);
            msg.pose.orientation = tf2::toMsg(q);

            pose_pub->publish(msg);
            matd_destroy(pose.R); 
            matd_destroy(pose.t);
        }
    }

    sensor_msgs::msg::CameraInfo::SharedPtr cam_info;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    apriltag_family_t * tf;
    apriltag_detector_t * td;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagDetector>());
    rclcpp::shutdown();
    return 0;
}