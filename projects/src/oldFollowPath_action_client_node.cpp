#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

class FollowPathActionClientNode : public rclcpp::Node
{
public:
    explicit FollowPathActionClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("FollowPath_action_client_node", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "follow_path");
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&FollowPathActionClientNode::send_goal, this));
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr client_ptr_;

    void send_goal()
    {
        timer_->cancel();
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        // Populate path_msg and publish path
        auto path_msg = createPath(); // Assuming you have a function to create the path message
        publishPath(path_msg);

        // Create and send goal
        auto goal_msg = nav2_msgs::action::FollowPath::Goal();
        goal_msg.path = path_msg;
        goal_msg.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FollowPathActionClientNode::onGoalResponse, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&FollowPathActionClientNode::onActionResult, this, std::placeholders::_1);
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    nav_msgs::msg::Path createPath()
    {
        // Create and return a sample path message
        nav_msgs::msg::Path path_msg;
        // Populate the path_msg sample for testing
        path_msg.header.frame_id = "map"; // Assuming the frame ID is "map"

        // Create some sample poses
        geometry_msgs::msg::PoseStamped pose1, pose2, pose3;
        pose1.pose.position.x = 0.0;
        pose1.pose.position.y = 0.0;
        pose1.pose.orientation.w = 1.0;

        pose2.pose.position.x = 1.0;
        pose2.pose.position.y = 1.0;
        pose2.pose.orientation.w = 1.0;

        pose3.pose.position.x = 2.0;
        pose3.pose.position.y = 2.0;
        pose3.pose.orientation.w = 1.0;

        // Add poses to the path
        path_msg.poses.push_back(pose1);
        path_msg.poses.push_back(pose2);
        path_msg.poses.push_back(pose3); 
        return path_msg;
    }

    void publishPath(const nav_msgs::msg::Path& path_msg)
    {
        auto path_publisher = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
        path_publisher->publish(path_msg);
    }

    void onGoalResponse(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
        }
    }

    void onActionResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Action succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Action aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Action canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown action result code");
                break;
        }
        rclcpp::shutdown();
    }

    
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    auto node = std::make_shared<FollowPathActionClientNode>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
