#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class VictimsPathPlannerNode: public rclcpp_lifecycle::LifecycleNode
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_obstacles_;

    void handlePolygonMsg(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received Polygon with %zu points:", msg->points.size());
        for (const auto& point : msg->points)
        {
            RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f", point.x, point.y);
        }
    }
    void handleObstacleArrayMsg(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received ObstacleArray with %zu obstacles:", msg->obstacles.size());
        for (const auto& obstacle : msg->obstacles)
        {
            RCLCPP_INFO(get_logger(), "  x: %.2f, y: %.2f", obstacle.polygon.points[0].x, obstacle.polygon.points[0].y);
            RCLCPP_INFO(get_logger(), "  radius: %.2f", obstacle.radius);
        }
    }

public:
  // set up configuration
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State& state)
  {
    // Create subscription to /map_borders
    this->sub_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", 10, 
      [this](const geometry_msgs::msg::Polygon::SharedPtr msg) {
         handlePolygonMsg(msg);
      }
    );
    RCLCPP_INFO(get_logger(), "Subscribed to map_borders");

    // Create subscription to /obstacles
    this->sub_obstacles_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/obstacles", 10, 
      [this](const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
         handleObstacleArrayMsg(msg);
      }
    );
    RCLCPP_INFO(get_logger(), "Subscribed to obstacles");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // on activate
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating VictimsPathPlannerNode");
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
  // Create the node 
  auto node = std::make_shared<VictimsPathPlannerNode>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
