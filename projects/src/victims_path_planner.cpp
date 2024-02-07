#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "edge_struct.hpp"

class VictimsPathPlannerNode: public rclcpp_lifecycle::LifecycleNode
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_victims_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gate_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_initial_pos_;

    const rclcpp::QoS qos = 10;

    bool borders_ready = false;
    bool obstacles_ready = false;
    bool victims_ready = false;
    bool gate_ready = false;
    bool initial_pose_ready = false;
    bool roadmap_ready = false;

    std::vector<edge> borders;
    std::vector<obstacle> obstacles;
    std::vector<edge> victims;
    edge gate_pose;
    edge initial_pose;

    void borders_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received borders with %zu points:", msg->points.size());
        for (const auto& point : msg->points)
        {
            RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f", point.x, point.y);
            this->borders.push_back({point.x, point.y});
        }
        this->borders_ready = true;
        this->activate_wrapper();
    }

    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received array with %zu obstacles:", msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles)
        {
            if (obstacle.polygon.points.size() == 1) { //cylinder
              RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f, radius: %.2f, type: cylinder", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
              this->obstacles.push_back({obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius});
            } else { // box
              // TODO
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
          this->gate_pose ={pose.position.x, pose.position.y};
        }
        this->gate_ready = true;
        this->activate_wrapper();
    }

    void initial_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received intial pose =>  x: %.2f, y: %.2f", msg->position.x, msg->position.y);
        this->initial_pose ={msg->position.x, msg->position.y};
        this->initial_pose_ready = true;
        this->activate_wrapper();
    }

    void activate_wrapper()
    {
      this->create_roadmap();
      if (this->roadmap_ready) {
        this->activate();
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Waiting for roadmap to be ready");
      }
    }

    void create_roadmap()
    {
      if (this->borders_ready && this->gate_ready && this->victims_ready && this->obstacles_ready) {
        //TODO

        this->roadmap_ready = true;
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

    // Create subscription to /shelfino#/initialpose
    this->declare_parameter("shelfino_id", 0);
    std::string initial_pose_topic = "/shelfino#/initialpose";
    initial_pose_topic.replace(initial_pose_topic.find('#'), 1, std::to_string(this->get_parameter("shelfino_id").as_int()));

    this->sub_initial_pos_ = this->create_subscription<geometry_msgs::msg::Pose>(
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
    //TODO: path planning
    //TODO: Nav2 FollowPath
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
