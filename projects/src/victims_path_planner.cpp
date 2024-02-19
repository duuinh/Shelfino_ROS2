#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "graph_node_struct.hpp"
#include "ilp_solver.hpp"
#include "common_defines.h"
#include "utils.h"
#include "nav_msgs/msg/path.hpp"
#include "orienteering_solver.hpp"
#include "a_star.hpp"

class VictimsPathPlannerNode: public rclcpp_lifecycle::LifecycleNode
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_victims_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gate_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pos_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    
    bool borders_ready = false;
    bool obstacles_ready = false;
    bool victims_ready = false;
    bool gate_ready = false;
    bool initial_pose_ready = false;
    bool roadmap_ready = false;

    std::vector<graph_node> borders;
    std::vector<obstacle> obstacles;
    std::vector<graph_node> victims;
    geometry_msgs::msg::Pose gate_pose;
    graph_node initial_pose;

    std::vector<std::vector<double>> distance_matrix;
    std::vector<std::vector<std::vector<graph_node>>> road_map;

    void construct_roadmap();

    void borders_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received borders with %zu points:", msg->points.size());
        for (const auto& point : msg->points)
        {
            RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f", point.x, point.y);
            this->borders.push_back({point.x, point.y, 0.0});
        }
        this->borders_ready = true;
        this->activate_wrapper();
    }

    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received array with %zu obstacles:", msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles)
        {
            if (obstacle.polygon.points.size() == 1) {
              RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f, radius: %.2f, type: cylinder", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
              this->obstacles.push_back({obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius, 0, 0, obstacle_type::CYLINDER});
            } else {
              double x = (obstacle.polygon.points[0].x + obstacle.polygon.points[2].x)/2.0;
              double y = (obstacle.polygon.points[0].y + obstacle.polygon.points[1].y)/2.0;
              double dx = (obstacle.polygon.points[2].x - x)*2;
              double dy = (obstacle.polygon.points[2].y - y)*2;
              this->obstacles.push_back({x, y, 0, dx, dy, obstacle_type::BOX});
            }
            
        }
        this->obstacles_ready = true;
        this->activate_wrapper();
    }

    void victims_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received array with %zu victims:", msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles)
        {
            RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f, weight: %.2f", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
            this->victims.push_back({obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius});
        }
        this->victims_ready = true;
        this->activate_wrapper();
    }

    void gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received gate ( %zu poses):", msg->poses.size());
        for (const auto& pose : msg->poses) {
          RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f", pose.position.x, pose.position.y);
          // obstacle gate {0.0, pose.position.x, pose.position.y, 1.0, 1.0, obstacle_type::BOX};
          this->gate_pose = pose;
        }
        this->gate_ready = true;
        this->activate_wrapper();
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received intial pose =>  x: %.2f, y: %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        this->initial_pose ={msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0};
        this->initial_pose_ready = true;
        this->activate_wrapper();
    }

    void activate_wrapper()
    {
      this->construct_roadmap();
      if (this->roadmap_ready) {
        RCLCPP_INFO(this->get_logger(), "Roadmap ready");
        this->activate();
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Waiting for roadmap to be ready");
      }
    }

public:
  explicit VictimsPathPlannerNode(bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode("victims_path_planner", 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)
    )
  {
    this->configure();
  } 
  // on configure
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State& state)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    // Create subscription to /map_borders
    this->sub_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", qos, 
      std::bind(&VictimsPathPlannerNode::borders_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(get_logger(), "Subscribed to map_borders");

    // Create subscription to /obstacles
    this->sub_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", qos, 
      std::bind(&VictimsPathPlannerNode::obstacles_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(get_logger(), "Subscribed to obstacles");

    // Create subscription to /victims
    this->sub_victims_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/victims", qos, 
      std::bind(&VictimsPathPlannerNode::victims_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(get_logger(), "Subscribed to victims");

    // Create subscription to /gate
    this->sub_gate_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gate_position", qos,
        std::bind(&VictimsPathPlannerNode::gate_callback, this, std::placeholders::_1)
      );
    RCLCPP_INFO(get_logger(), "Subscribed to gate");

    // Create subscription to /shelfino#/amcl_pose
    this->declare_parameter("shelfino_id", 0);
    std::string initial_pose_topic = "/shelfino#/amcl_pose";
    initial_pose_topic.replace(initial_pose_topic.find('#'), 1, std::to_string(this->get_parameter("shelfino_id").as_int()));

    this->sub_initial_pos_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic.c_str(), qos,
        std::bind(&VictimsPathPlannerNode::initial_pose_callback, this, std::placeholders::_1)
      );
    RCLCPP_INFO(get_logger(), ("Subscribed to " + initial_pose_topic).c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // on activate
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating VictimsPathPlannerNode");
    this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("victims_rescue_path", 10);
    double max_distance = ROBOT_VELOCITY * MAX_TIME;
    std::vector<double> rewards;

    // initilize rawards vector
    rewards.push_back(0);   // start node
    for (size_t i = 0; i < victims.size(); i++) {
      rewards.push_back(victims[i].reward);
    }
    rewards.push_back(0);   // end node

    auto start_time = get_clock()->now().seconds();
    RCLCPP_INFO(get_logger(), "Starting mission planning");
    // Brute force
    std::vector<int> path = find_optimal_path(max_distance, distance_matrix, rewards);

    // // Branch and Bound method
    // ILP_Solver solver = ILP_Solver(rewards, road_map, max_distance);
    // std::vector<int> path = solver.find_optimal_path_BnB();

    RCLCPP_INFO(get_logger(), "Finished mission planning [time: %f sec]", get_clock()->now().seconds() - start_time);

    std::stringstream ss;
    ss << "Optimal path: ";

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    for (int i = 0; i < path.size(); ++i) {   
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = now();
        pose_stamped.header.frame_id = "map";

        if (i == path.size() - 1) {
            pose_stamped.pose.position.x = gate_pose.position.x;
            pose_stamped.pose.position.y = gate_pose.position.y;
            pose_stamped.pose.position.z = gate_pose.position.z;
            pose_stamped.pose.orientation.x = gate_pose.orientation.x;
            pose_stamped.pose.orientation.y = gate_pose.orientation.y;
            pose_stamped.pose.orientation.z = gate_pose.orientation.z;
            pose_stamped.pose.orientation.w = gate_pose.orientation.w;
        } else if (i == 0) {
            pose_stamped.pose.position.x = initial_pose.x;
            pose_stamped.pose.position.y = initial_pose.y;
        } else {
            pose_stamped.pose.position.x = victims[i-1].x;
            pose_stamped.pose.position.y = victims[i-1].y;
        }

        path_msg.poses.push_back(pose_stamped);

        ss<< path[i]<<", ";
    }
    
    RCLCPP_INFO(get_logger(), ss.str().c_str(), get_clock()->now().seconds());
    this->path_publisher_->publish(path_msg);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // on deactivate
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating VictimsPathPlannerNode");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

void VictimsPathPlannerNode::construct_roadmap() {
  if (this->borders_ready && this->gate_ready && this->victims_ready && this->obstacles_ready && this->initial_pose_ready) {
    auto start_time = get_clock()->now().seconds();
    RCLCPP_INFO(get_logger(), "Starting roadmap construction");

    std::vector<graph_node> nodes = this->victims;
    nodes.insert(nodes.begin(), initial_pose);
    nodes.push_back({gate_pose.position.x, gate_pose.position.y});

    distance_matrix.resize(nodes.size(), std::vector<double>(nodes.size(), 0));
    road_map.resize(nodes.size(), std::vector<std::vector<graph_node>>(nodes.size()));

    AStar planner = AStar(borders, obstacles);
    for (size_t i = 0; i < nodes.size(); ++i) {
      for (size_t j = 0; j < nodes.size(); ++j) {
          if (i != j) {
            ShortestPath shortest_path = planner.get_shortest_path(nodes[i],nodes[j]);
            distance_matrix[i][j] = shortest_path.length;
            road_map[i][j] = shortest_path.path;
          }
      }
    }

    RCLCPP_INFO(get_logger(), "Finished roadmap construction [time: %f sec]", get_clock()->now().seconds()-start_time);

    // print log
    for (size_t i = 0; i < nodes.size(); ++i) {
      std::string log = "  [ ";
      for (size_t j = 0; j < nodes.size(); ++j) {
          log += std::to_string(distance_matrix[i][j]) + " ";
      }
      log += "]";
      RCLCPP_INFO(get_logger(), log.c_str() );
    }
    this->roadmap_ready = true;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // create node 
  auto node = std::make_shared<VictimsPathPlannerNode>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
