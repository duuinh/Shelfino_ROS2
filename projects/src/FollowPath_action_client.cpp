#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace projects
{
class FollowPathActionClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit FollowPathActionClient(const rclcpp::NodeOptions & options)
  : Node("FollowPath_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<FollowPath>(
      this,
      "follow_path");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FollowPathActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = FollowPath::Goal();
    
    // Sample path creation
    nav_msgs::msg::Path path;
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = i * 0.5; // X coordinate increases by 0.5 for each pose
        pose.pose.position.y = 0.0;     // Y coordinate remains constant at 0
        pose.pose.orientation.w = 1.0;   // Quaternion representing no rotation
        path.poses.push_back(pose);
    }

    // Setting the path and other fields in the goal message
    goal_msg.path = path;
    // goal_msg.controller_id = "default_controller";
    // goal_msg.goal_checker_id = "default_goal_checker";
    //goal_msg.progress_checker_id = "default_progress_checker";


    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FollowPathActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FollowPathActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FollowPathActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

  }

private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleFollowPath::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
  {
    auto distance_feedback_msg = std_msgs::msg::String();
    distance_feedback_msg.data = "Distance to goal: " + std::to_string(feedback->distance_to_goal);
    RCLCPP_INFO(this->get_logger(), distance_feedback_msg.data);
  }

  void result_callback(const GoalHandleFollowPath::WrappedResult & result)
  {
    switch (result.code) {
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
    RCLCPP_INFO(this->get_logger(), "Result received");
    rclcpp::shutdown();
  }
};  

}  

//RCLCPP_COMPONENTS_REGISTER_NODE(projects::FollowPathActionClient)

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS 2 node

    // Create a ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("follow_path_action_client_node");

    // Create an instance of FollowPathActionClient node
    auto follow_path_client = std::make_shared<projects::FollowPathActionClient>(node->get_node_options());

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown(); // Shutdown ROS 2 node

    return 0;
}
