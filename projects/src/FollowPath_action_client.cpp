#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "dubins.hpp"

using namespace std::chrono_literals;


// Function to convert Euler angles to quaternion
geometry_msgs::msg::Quaternion convert_to_quaternion(double pitch, double roll, double yaw)
{
  double cp = cos(pitch/2);
  double sp = sin(pitch/2);
  double cr = cos(roll/2);
  double sr = sin(roll/2);
  double cy = cos(yaw/2);
  double sy = sin(yaw/2);

  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = sp * cr * cy - cp * sr * sy;
  q_msg.y = cp * sr * cy + sp * cr * sy;
  q_msg.z = cp * cr * sy - sp * sr * cy;
  q_msg.w = cp * cr * cy + sp * sr * sy;
  
  return q_msg;
}

// Function to generate a path from Dubins curves
nav_msgs::msg::Path generate_path(std::vector<PathCurve> curves, std::msgs::msg::Header header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.header.frame_id = "map";

  for (auto curve : curves)
  {
    auto points = curve.toPointsUniform(0.1);
    for (auto point : points)
    {
      pose.pose.position.x = point.xPos;
      pose.pose.position.y = point.yPos;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = convert_to_quaternion(0, 0, point.orientation);
      path.poses.push_back(pose);
    }
  }

  return path;
}


// Function to split a path into smaller sections of a specified distance
std::vector<nav_msgs::msg::Path> split_path(std::vector<PathCurve> curves, std::msgs::msg::Header header, double section_length) {
    nav_msgs::msg::Path path = generate_path(curves, header);//need to add the path, header
    std::vector<nav_msgs::msg::Path> sections;
    nav_msgs::msg::Path current_section;
    double accumulated_distance = 0.0;

    for (const auto& pose : path.poses) {
        // If the current section is empty, add the pose
        if (current_section.poses.empty()) {
            current_section.header = path.header;
            current_section.header.frame_id = "map";
            current_section.poses.push_back(pose);
        } else {
            // Calculate the distance between the current pose and the last pose in the section
            auto& last_pose = current_section.poses.back();
            double dx = pose.pose.position.x - last_pose.pose.position.x;
            double dy = pose.pose.position.y - last_pose.pose.position.y;
            double dz = pose.pose.position.z - last_pose.pose.position.z;
            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            // If adding the pose exceeds the section distance, start a new section
            if (accumulated_distance + distance > section_length) {
                sections.push_back(current_section);
                current_section = nav_msgs::msg::Path();
                current_section.header = path.header;
                current_section.header.frame_id = "map";
                current_section.poses.push_back(pose);
                accumulated_distance = 0.0;
            } else {
                current_section.poses.push_back(pose);
                accumulated_distance += distance;
            }
        }
    }

    // Add the last section if it's not empty
    if (!current_section.poses.empty()) {
        sections.push_back(current_section);
    }

    return sections;
}


class PathPub : public rclcpp::Node
{
public:
  PathPub() : Node("path_pub")
  {
    // Get shelfino ID parameter
    this->declare_parameter<int>("shelfino_id", 0);
    this->get_parameter("shelfino_id", shelfino_id_);
    std::string namespace_ = "/shelfino" + std::to_string(shelfino_id_);

    // TO DO: maybe directly call the generate path function to get the path?
    // TO DO: call the split path function to get the sections

    // Create publisher for the path
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(namespace_ + "/plan1", 10);

    // Start publishing loop
    publish_path_timer_ = this->create_wall_timer(
      1000ms, std::bind(&PathPub::publish_path_callback, this));
  }

private:
  void publish_path_callback()
  {
    if(!path.has_value())
    {
      RCLCPP_ERROR(this->get_logger(), "Path not available");
      return;
    }
  
    // TO DO: must publish the sections one by one   
    std_msgs::msg::Header header;
    path_publisher_->publish(path_msg);           
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr publish_path_timer_;
};




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

  
    std::string topic_name = namespace_ + "/plan1";

    // Subscribe to the /plan1 topic
    plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(topic_name, 10, std::bind(&FollowPathActionClient::plan_callback, this, std::placeholders::_1));

  }

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
    goal_msg.path = path_;
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