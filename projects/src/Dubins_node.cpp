#include "dubins.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"



class DubinsNode : public rclcpp::Node
{
public:
    DubinsNode() : Node("dubins_node")
    {
        // Get shelfino ID parameter
        this->declare_parameter<int>("shelfino_id", 0);
        this->get_parameter("shelfino_id", shelfino_id_);
        std::string namespace_ = "/shelfino" + std::to_string(shelfino_id_);
        
        std::string topic_name = namespace_ + "/plan1";
        // Subscribe to the path topic
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path_topic", 10, std::bind(&DubinsNode::pathCallback, this, std::placeholders::_1));

        // Publisher for the Dubins path
        dubins_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dubins_path_publisher_;
    double curvature = 1.0; // Set your curvature value here
    Polygon obstaclePolygon; // Initialize your obstaclePolygon object here

    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        // Vector to store Dubins curves for the entire trajectory
        std::vector<PathCurve> dubins_curves;

        // Iterate over pairs of consecutive points from the received path
        for (size_t i = 0; i < path_msg->poses.size() - 1; ++i)
        {
            // Get start and end points for the Dubins path calculation
            const auto &start_pose = path_msg->poses[i].pose;
            const auto &end_pose = path_msg->poses[i + 1].pose;

            PathPoint start_point(start_pose.position.x, start_pose.position.y, start_pose.position.z);
            PathPoint end_point(end_pose.position.x, end_pose.position.y, end_pose.position.z);

            // Calculate Dubins path between the consecutive points
            auto dubins_curve = findShortestPathCurve(start_point, end_point, curvature, obstaclePolygon);

            // Append Dubins curve to the vector
            dubins_curves.push_back(dubins_curve);
        }

        auto header = std_msgs::msg::Header();

        // Convert Dubins curves to a sequence of points
        auto dubins_path = generate_path(dubins_curves, header);

        // Publish the Dubins path
        dubins_path_publisher_->publish(dubins_path);
    }
};


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
std::vector<nav_msgs::msg::Path> split_path(const nav_msgs::msg::Path& dubins_path, double section_length) {
    std::vector<nav_msgs::msg::Path> sections;
    nav_msgs::msg::Path current_section;
    double accumulated_distance = 0.0;

    for (const auto& pose : dubins_path.poses) {
        // If the current section is empty, add the pose
        if (current_section.poses.empty()) {
            current_section.header = dubins_path.header;
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
                current_section.header = dubins_path.header;
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



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DubinsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
