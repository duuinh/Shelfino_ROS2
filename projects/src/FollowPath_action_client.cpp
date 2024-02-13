#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "dubins.hpp"

using namespace std::chrono_literals;




class FollowPathActionClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit FollowPathActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("FollowPath_action_client", options)
  {
    // Get shelfino ID parameter
    this->declare_parameter<int>("shelfino_id", 0);
    this->get_parameter("shelfino_id", shelfino_id_);
    std::string namespace_ = "/shelfino" + std::to_string(shelfino_id_);
    
    std::string action_server_name = namespace_ + "/follow_path";
    this->client_ptr_ = rclcpp_action::create_client<FollowPath>(
      this, action_server_name);

    this->timer_ = this->create_wall_timer(
      500ms,
      std::bind(&FollowPathActionClient::send_goal, this));

  
    // std::string topic_name = namespace_ + "/plan1";

    // // Subscribe to the /plan1 topic
    // plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(topic_name, 10, std::bind(&FollowPathActionClient::plan_callback, this, std::placeholders::_1));

  }

  // callback "next_goal" to iterate over the path sections and send each section as goal to action server one by one
  // in feedback callback, check if the distance to goal is less than a threshold, if so stop communication with the server
  // in result callback, check if the result is succeeded, then send the next goal

  


  void send_goal()
  {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    
    auto goal_msg = FollowPath::Goal();
    goal.msg.header.stamp = this->now();
    goal.msg.header.frame_id = "map";

    // TO DO: must call the split path function to get the sections of path
    
    goal_msg.path = generate_path(curves, header);

    goal_msg.controller_id = "FollowPath";
    

   
    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FollowPathActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&FollowPathActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&FollowPathActionClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sending goal");



  }

private:
  
  void goal_response_callback(const GoalHandleFollowPath::SharedPtr& goal_handle)
  {
    //auto goal_handle = future.get();
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
    std::stringstream ss;
    ss << "Distance to goal: " << feedback->distance_to_goal << ", Speed: " << feedback->speed;
    RCLCPP_INFO(this->get_logger(), "Received feedback");
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  // Subscription callback function for /plan1 topic
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // Store the received path
    path_ = *msg;
    //path_.header.frame_id = "map"; // Set the frame ID of the path
    RCLCPP_INFO(this->get_logger(), "Received path");
  }


  void result_callback(const GoalHandleFollowPath::WrappedResult & result)
  {
    switch(result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  nav_msgs::msg::Path path_; // Declare path_ variable here
  int shelfino_id_; 
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<FollowPathActionClient>();

    rclcpp::spin(action_client);
    rclcpp::shutdown();

    return 0;
}