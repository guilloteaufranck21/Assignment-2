#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navigator_client
{

    class ActionClient2 : public rclcpp::Node
    {
    public:
        ActionClient2()
            : Node("action_client_2")
        {
            // We subscrbe to /goal_between_tags topic where position between tags is written by goal_calculator node
            this->x = 0.0;
            this->y = 0.0;

            auto topic_callback =
                [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void
            {
                this->x = msg->pose.position.x;
                this->y = msg->pose.position.x;
            };
            subscription_ =
                this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_between_tags", 10, topic_callback);
        }

        using Dialog = nav2_msgs::action::NavigateToPose;
        using GoalHandleDialog = rclcpp_action::ClientGoalHandle<Dialog>;

        explicit ActionClient2(const rclcpp::NodeOptions &options)
            : Node("Action_client_2", options), distance_remaining_(0.0), time_remaining_(0)
        {
            this->client_ptr_ = rclcpp_action::create_client<Dialog>(
                this,
                "navigate_to_pose"); // Action that takes destination's position

            auto timer_callback_lambda = [this]()
            { return this->send_goal(); };
            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                timer_callback_lambda);

            this->publish_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]()
                {
                    RCLCPP_INFO(this->get_logger(), "Distance remaining: %f m", distance_remaining_);
                    RCLCPP_INFO(this->get_logger(), "Estimated time remaining: %d s", time_remaining_);
                });
        }

        void send_goal()
        {
            using namespace std::placeholders;

            this->timer_->cancel();

            if (!this->client_ptr_->wait_for_action_server())
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = Dialog::Goal();
            geometry_msgs::msg::Pose pose;
            geometry_msgs::msg::Point point;
            geometry_msgs::msg::Quaternion quat;
            geometry_msgs::msg::PoseStamped poseStamp;
            std_msgs::msg::Header header;

            header.frame_id = "map";
            header.stamp = this->get_clock()->now();
            // Values obtained through /goal_between_tags topic that we subscribed to earlier
            point.x = this->x;
            point.y = this->y;
            point.z = 0.0;

            quat.x = 0.0;
            quat.y = 0.0;
            quat.z = 0.0;
            quat.w = 1.0;

            pose.position = point;
            pose.orientation = quat;

            poseStamp.pose = pose;
            poseStamp.header = header;

            goal_msg.pose = poseStamp;

            RCLCPP_INFO(this->get_logger(), "Sending goal with values (x=%f;y=%f)", point.x, point.y);

            auto send_goal_options = rclcpp_action::Client<Dialog>::SendGoalOptions();
            send_goal_options.goal_response_callback = [this](const GoalHandleDialog::SharedPtr &goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            };

            send_goal_options.feedback_callback = [this](
                                                      GoalHandleDialog::SharedPtr,
                                                      const std::shared_ptr<const Dialog::Feedback> feedback)
            {
                // Time and distance remaining until destination reached obtained through action server feedback
                distance_remaining_ = feedback->distance_remaining;
                time_remaining_ = feedback->estimated_time_remaining.sec;
            };

            send_goal_options.result_callback = [this](const GoalHandleDialog::WrappedResult &result)
            {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
                }
                std::stringstream ss;
                ss << "Robot arrived at destination";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                rclcpp::shutdown();
            };
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
        float x;
        float y;
        rclcpp_action::Client<Dialog>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        float distance_remaining_;
        int time_remaining_;
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigator_client::ActionClient2>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(navigator_client::ActionClient2)