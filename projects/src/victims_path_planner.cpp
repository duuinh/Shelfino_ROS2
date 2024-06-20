#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "common_struct.hpp"
#include "ilp_solver.hpp"
#include "common_defines.hpp"
#include "utils.h"
#include "nav_msgs/msg/path.hpp"
#include "orienteering_solver.hpp"
#include "prm.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
class VictimsPathPlannerNode : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_borders_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_victims_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gate_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pos_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  bool borders_ready = false;
  bool obstacles_ready = false;
  bool victims_ready = false;
  bool gate_ready = false;
  bool initial_pose_ready = false;
  bool roadmap_ready = false;

  std::vector<GraphNode> borders;
  std::vector<Obstacle> obstacles;
  std::vector<GraphNode> victims;
  geometry_msgs::msg::Pose gate_pose;
  geometry_msgs::msg::Pose initial_pose;

  std::vector<std::vector<double>> distance_matrix;
  std::vector<std::vector<std::vector<GraphNode>>> road_map;

  void construct_roadmap();

  void borders_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received borders with %lu points:", msg->points.size());
    for (const auto &point : msg->points)
    {
      RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f", point.x, point.y);
      this->borders.push_back({point.x, point.y, 0.0});
    }
    this->borders_ready = true;
    this->activate_wrapper();
  }

  void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received array with %lu obstacles:", msg->obstacles.size());
    for (const auto &obstacle : msg->obstacles)
    {
      if (obstacle.polygon.points.size() == 1)
      {
        RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f, radius: %.2f, type: cylinder", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
        this->obstacles.push_back({obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius, 0, 0, ObstacleType::CYLINDER});
      }
      else
      {
        double x = (obstacle.polygon.points[0].x + obstacle.polygon.points[2].x) / 2.0;
        double y = (obstacle.polygon.points[0].y + obstacle.polygon.points[1].y) / 2.0;
        double dx = (obstacle.polygon.points[2].x - x) * 2;
        double dy = (obstacle.polygon.points[2].y - y) * 2;
        this->obstacles.push_back({x, y, 0, dx, dy, ObstacleType::BOX});
      }
    }
    this->obstacles_ready = true;
    this->activate_wrapper();
  }

  void victims_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received array with %lu victims:", msg->obstacles.size());
    for (const auto &obstacle : msg->obstacles)
    {
      RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f, weight: %.2f", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
      this->victims.push_back({obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius});
    }
    this->victims_ready = true;
    this->activate_wrapper();
  }

  void gate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received gate ( %lu poses):", msg->poses.size());
    for (const auto &pose : msg->poses)
    {
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
    this->initial_pose = msg->pose.pose;
    this->initial_pose_ready = true;
    this->activate_wrapper();
  }

  void activate_wrapper()
  {
    if (!this->roadmap_ready)
    {
      this->construct_roadmap();
    }
  }

  void visualize_roadmap_nodes(std::vector<Point> nodes) {
        RCLCPP_INFO(get_logger(), "Total nodes in roadmap: %lu", nodes.size());
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "map";
        visualization_msgs::msg::MarkerArray markers;
        for (size_t i = 0; i < nodes.size(); ++i) {
            // add marker
            visualization_msgs::msg::Marker pos_marker;
            pos_marker.header = header;
            pos_marker.ns = "roadmap_nodes";
            pos_marker.id = i;
            pos_marker.action = visualization_msgs::msg::Marker::ADD;
            pos_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            pos_marker.pose.position.x = nodes[i].x;
            pos_marker.pose.position.y = nodes[i].y;
            pos_marker.scale.x = 0.1;
            pos_marker.scale.y = 0.1;
            pos_marker.scale.z = 0.1;
            pos_marker.color.a = 0.5;
            pos_marker.color.r = 1.0;
            pos_marker.color.g = 0.0;
            pos_marker.color.b = 1.0;
            markers.markers.push_back(pos_marker);
        }

        this->marker_publisher_->publish(markers);
    }

public:
  explicit VictimsPathPlannerNode(bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode("victims_path_planner",
                                        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    this->configure();
  }
  // on configure
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
    // Create subscription to /map_borders
    this->sub_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "/map_borders", qos,
        std::bind(&VictimsPathPlannerNode::borders_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to map_borders");

    // Create subscription to /obstacles
    this->sub_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", qos,
        std::bind(&VictimsPathPlannerNode::obstacles_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to obstacles");

    // Create subscription to /victims
    this->sub_victims_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/victims", qos,
        std::bind(&VictimsPathPlannerNode::victims_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to victims");

    // Create subscription to /gate
    this->sub_gate_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gate_position", qos,
        std::bind(&VictimsPathPlannerNode::gate_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to gate");

    // Create subscription to /shelfino#/amcl_pose
    this->declare_parameter("shelfino_id", 0);
    std::string initial_pose_topic = "/shelfino#/amcl_pose";
    initial_pose_topic.replace(initial_pose_topic.find('#'), 1, std::to_string(this->get_parameter("shelfino_id").as_int()));

    this->sub_initial_pos_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic.c_str(), qos,
        std::bind(&VictimsPathPlannerNode::initial_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), ("Subscribed to " + initial_pose_topic).c_str());

    this->marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers/path_points", rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom));
    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // on activate
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating VictimsPathPlannerNode");
    this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("victims_rescue_path", 10);
    double max_distance = ROBOT_VELOCITY * MAX_TIME;
    std::vector<double> rewards;

    // initilize rawards vector
    rewards.push_back(0); // start node
    for (size_t i = 0; i < victims.size(); i++)
    {
      rewards.push_back(victims[i].reward);
    }
    rewards.push_back(0); // end node

    auto start_time = get_clock()->now().seconds();
    RCLCPP_INFO(get_logger(), "Starting mission planning");

    // Brute force
    std::vector<int> node_indices = find_optimal_path(max_distance, distance_matrix, rewards);

    // // Branch and Bound method
    // ILP_Solver solver = ILP_Solver(rewards, road_map, max_distance);
    // std::vector<int> node_indices = solver.find_optimal_path_BnB();

    std::stringstream ss;
    ss << "Optimal path: ";
    for (size_t i = 0; i < node_indices.size(); ++i)
    {
      ss << node_indices[i] << ", ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());

    RCLCPP_INFO(get_logger(), "Finished mission planning [time: %f sec]", get_clock()->now().seconds() - start_time);

    // get actual path from roadmap
    std::vector<GraphNode> path;
    for (size_t i = 0; i < node_indices.size() - 1; ++i)
    {
      std::vector<GraphNode> edge = road_map[node_indices.at(i)][node_indices.at(i + 1)];
      if (!edge.empty()) {
        path.insert(path.end(), edge.begin(), edge.end() - 1);
      } else {
           RCLCPP_ERROR(get_logger(), "Edge from %u to %u is empty", node_indices.at(i), node_indices.at(i + 1));
      }
    }

    // sending path msg to action client node
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    for (size_t i = 0; i < path.size() + 1; ++i)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = now();
      pose_stamped.header.frame_id = "map";

      if (i == path.size())
      {
        pose_stamped.pose.position.x = gate_pose.position.x;
        pose_stamped.pose.position.y = gate_pose.position.y;
        pose_stamped.pose.position.z = gate_pose.position.z;
        pose_stamped.pose.orientation.x = gate_pose.orientation.x;
        pose_stamped.pose.orientation.y = gate_pose.orientation.y;
        pose_stamped.pose.orientation.z = gate_pose.orientation.z;
        pose_stamped.pose.orientation.w = gate_pose.orientation.w;
      }
      else if (i == 0)
      {
        pose_stamped.pose.position.x = initial_pose.position.x;
        pose_stamped.pose.position.y = initial_pose.position.y;
        pose_stamped.pose.orientation.x = initial_pose.orientation.x;
        pose_stamped.pose.orientation.y = initial_pose.orientation.y;
        pose_stamped.pose.orientation.z = initial_pose.orientation.z;
        pose_stamped.pose.orientation.w = initial_pose.orientation.w;
      }
      else
      {
        pose_stamped.pose.position.x = path[i].x;
        pose_stamped.pose.position.y = path[i].y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
      }
      path_msg.poses.push_back(pose_stamped);
    }

    this->path_publisher_->publish(path_msg);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // on deactivate
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating VictimsPathPlannerNode");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

void VictimsPathPlannerNode::construct_roadmap()
{
  if (this->borders_ready && this->gate_ready && this->victims_ready && this->obstacles_ready && this->initial_pose_ready)
  {
    auto start_time = get_clock()->now().seconds();
    RCLCPP_INFO(get_logger(), "Starting roadmap construction");

    std::vector<GraphNode> nodes = this->victims;
    nodes.insert(nodes.begin(), {initial_pose.position.x, initial_pose.position.y});
    nodes.push_back({gate_pose.position.x, gate_pose.position.y});

    distance_matrix.resize(nodes.size(), std::vector<double>(nodes.size(), 0));
    road_map.resize(nodes.size(), std::vector<std::vector<GraphNode>>(nodes.size()));
    PRM prm = PRM(obstacles, borders, nodes);
    visualize_roadmap_nodes(prm.get_roadmap().nodes);

    for (size_t i = 0; i < nodes.size(); ++i)
    {
      for (size_t j = 0; j < nodes.size(); ++j)
      {
        if (i != j)
        {
          RCLCPP_INFO(get_logger(), "find_shortest_path");
          ShortestPath shortest_path = prm.find_shortest_path({nodes[i].x, nodes[i].y}, {nodes[j].x, nodes[j].y});
          distance_matrix[i][j] = shortest_path.length;
          road_map[i][j] = shortest_path.path;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Finished roadmap construction [time: %f sec]", get_clock()->now().seconds() - start_time);

    // print log
    for (size_t i = 0; i < nodes.size(); ++i)
    {
      std::string log = "  [ ";
      for (size_t j = 0; j < nodes.size(); ++j)
      {
        log += std::to_string(distance_matrix[i][j]) + " ";
      }
      log += "]";
      RCLCPP_INFO(get_logger(), log.c_str());
    }
    this->roadmap_ready = true;
    RCLCPP_INFO(this->get_logger(), "Roadmap ready");
    this->activate();
  }
}

int main(int argc, char *argv[])
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
