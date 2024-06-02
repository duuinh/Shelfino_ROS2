#include "dubins.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"
#include "utils.h"

using namespace std::chrono_literals;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

// Function to convert Euler angles to quaternion
geometry_msgs::msg::Quaternion convert_to_quaternion(double pitch, double roll, double yaw) {
    double cp = cos(pitch / 2);
    double sp = sin(pitch / 2);
    double cr = cos(roll / 2);
    double sr = sin(roll / 2);
    double cy = cos(yaw / 2);
    double sy = sin(yaw / 2);

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = sp * cr * cy - cp * sr * sy;
    q_msg.y = cp * sr * cy + sp * cr * sy;
    q_msg.z = cp * cr * sy - sp * sr * cy;
    q_msg.w = cp * cr * cy + sp * sr * sy;

    return q_msg;
}

double get_yaw_angle(geometry_msgs::msg::Quaternion q_msg) {
    double siny_cosp = 2 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y);
    double cosy_cosp = 1 - 2 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    return yaw;
}

// Function to generate a path from Dubins curves
nav_msgs::msg::Path generate_path(std::vector<DubinsCurve> curves, std_msgs::msg::Header header) {
    nav_msgs::msg::Path path;
    path.header = header;
    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.header.frame_id = "map";

    for (auto curve : curves) {
        auto points = curve.toPointsUniform(0.1);
        for (auto point : points) {
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
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

class FollowPathActionClient : public rclcpp::Node {

   private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    int section_cnt = 0;
    std::vector<nav_msgs::msg::Path> sections;

    void send_goal(nav_msgs::msg::Path path) {
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FollowPath::Goal();
        goal_msg.path = path;
        goal_msg.path.header.stamp = this->now();
        goal_msg.path.header.frame_id = "map";
        goal_msg.controller_id = "FollowPath";
        goal_msg.goal_checker_id = "goal_checker";

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FollowPathActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&FollowPathActionClient::feedback_callback, this, std::placeholders::_1,
                      std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&FollowPathActionClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sending goal");
    }

    void goal_response_callback(const GoalHandleFollowPath::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(const GoalHandleFollowPath::SharedPtr& goal_handle,
                           const std::shared_ptr<const FollowPath::Feedback> feedback) {
        std::stringstream ss;
        double distance_to_goal = feedback->distance_to_goal;
        ss << "Distance to goal: " << distance_to_goal << ", Speed: " << feedback->speed;
        RCLCPP_INFO(this->get_logger(), "Received feedback");
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    
        // if (distance_to_goal < 0.5) {
        //     // this->publish_path(this->sections[section_cnt]);
        //     this->client_ptr_->async_cancel_all_goals
        //     (
        //     [this](rclcpp_action::Client<FollowPath>::SharedPtr client) {
        //         RCLCPP_INFO(get_logger(), "All goals canceled");
        //         this->publish_path(this->sections[section_cnt]);
        //     });
        // }
    }

    void result_callback(const GoalHandleFollowPath::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                 this->publish_path(this->sections[section_cnt]);
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

    void publish_path(nav_msgs::msg::Path path_msg) {
        if (this->section_cnt < this->sections.size()) {
            this->path_publisher_->publish(path_msg);
            this->send_goal(path_msg);
            this->section_cnt += 1;
            RCLCPP_INFO(this->get_logger(), "Section sent!");
        }
    }

    void plan_callback(nav_msgs::msg::Path::SharedPtr path_msg) {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        double section_length = 20.0;
        double arc_radius = 0.5;
        double max_curvature = 1.0/arc_radius;

        std::vector<DubinsCurve> dubins_curves;
        std::vector<WayPoint> points;

        auto start_time = get_clock()->now().seconds();
        RCLCPP_INFO(get_logger(), "Starting Dubins motion planning");
        RCLCPP_INFO(get_logger(), std::to_string(path_msg->poses.size()).c_str());

        // Iterate over pairs of consecutive points from the received path
        for (size_t i = 0; i < path_msg->poses.size(); ++i) {
            const auto &pose = path_msg->poses[i].pose;
            double orientation = get_yaw_angle(pose.orientation);
            WayPoint point(pose.position.x, pose.position.y, orientation);

            points.push_back(point);
        }

        dubins_curves = solve_multipoints_dubins(points, max_curvature);

        RCLCPP_INFO(get_logger(), "Finished Dubins motion planning [time: %f sec]", get_clock()->now().seconds() - start_time);

        // Convert Dubins curves to a sequence of points
        auto dubins_points = generate_path(dubins_curves, header);

        // Split the Dubins path into smaller sections
        this->sections = split_path(dubins_points, section_length); 

        this->publish_path(this->sections[section_cnt]);
    }

   public:
    explicit FollowPathActionClient() : Node("follow_path_action_client") {

        // Get shelfino ID parameter
        this->declare_parameter("shelfino_id", 0);

        std::string action_server_name = "/shelfino#/follow_path";
        action_server_name.replace(action_server_name.find('#'), 1,
                                   std::to_string(this->get_parameter("shelfino_id").as_int()));

        std::string publish_path_topic = "/shelfino#/plan1";
        publish_path_topic.replace(publish_path_topic.find('#'), 1,
                                   std::to_string(this->get_parameter("shelfino_id").as_int()));

        // create action client
        this->client_ptr_ = rclcpp_action::create_client<FollowPath>(this, action_server_name);
        // Create publisher for the path
        this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(publish_path_topic, 10);

        // Subscribe to the /victims_path_planner topic
        this->plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>("victims_rescue_path",
                 rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&FollowPathActionClient::plan_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<FollowPathActionClient>();

    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}