#include "my_custom_layers/my_csv_layer.hpp"
#include <nav2_util/node_utils.hpp>
#include <algorithm>

namespace my_custom_layers
{

MyCSVLayer::MyCSVLayer()
: enabled_(true)
{
}

MyCSVLayer::~MyCSVLayer()
{
}

void MyCSVLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("MyCSVLayer"), "Failed to lock lifecycle node!");
    return;
  }

  std::string param_topic = name_ + ".topic";
  nav2_util::declare_parameter_if_not_declared(
    node, param_topic, rclcpp::ParameterValue(std::string("/my_csv_data")));
  topic_name_ = node->get_parameter(param_topic).as_string();

  std::string param_enabled = name_ + ".enabled";
  nav2_util::declare_parameter_if_not_declared(
    node, param_enabled, rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(param_enabled).as_bool();

  rclcpp::SubscriptionOptions sub_opt;
  csv_sub_ = node->create_subscription<obstacle_influence::msg::CSVData>(
    topic_name_,
    rclcpp::QoS(10),
    std::bind(&MyCSVLayer::csvDataCallback, this, std::placeholders::_1),
    sub_opt
  );

  MyCSVLayer::matchSize();

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "[MyCSVLayer] onInitialize done. Subscribing to topic: %s", topic_name_.c_str());
}

void MyCSVLayer::csvDataCallback(const obstacle_influence::msg::CSVData::SharedPtr msg)
{
  enabled_ = msg->enabled;

  std::vector<std::tuple<double, double, unsigned char>> updated_data;
  updated_data.reserve(msg->points.size());

  for (auto & pt : msg->points) {
    int cost = pt.cost;
    if (cost < 0) cost = 0;
    if (cost > 255) cost = 255;
    updated_data.emplace_back(pt.x, pt.y, static_cast<unsigned char>(cost));
  }

  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
    csv_cost_data_ = std::move(updated_data);
  }

  RCLCPP_INFO(rclcpp::get_logger("MyCSVLayer"),
    "[csvDataCallback] Received %ld points, enabled=%d",
    msg->points.size(), msg->enabled);
}

void MyCSVLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y,
                              double* max_x, double* max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;

  if (!enabled_) {
    return;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  for (auto & data : csv_cost_data_) {
    double x = std::get<0>(data);
    double y = std::get<1>(data);
    *min_x = std::min(*min_x, x);
    *min_y = std::min(*min_y, y);
    *max_x = std::max(*max_x, x);
    *max_y = std::max(*max_y, y);
  }
}

void MyCSVLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  for (auto & data : csv_cost_data_) {
    double wx = std::get<0>(data);
    double wy = std::get<1>(data);
    unsigned char cost = std::get<2>(data);

    unsigned int mx, my;
    if (master_grid.worldToMap(wx, wy, mx, my)) {
      if (mx >= (unsigned int)min_i && mx < (unsigned int)max_i &&
          my >= (unsigned int)min_j && my < (unsigned int)max_j)
      {
        unsigned char old_cost = master_grid.getCost(mx, my);
        unsigned char new_cost = std::max(old_cost, cost);
        master_grid.setCost(mx, my, new_cost);
      }
    }
  }
}

void MyCSVLayer::reset()
{
  enabled_ = true;
}

}  // namespace my_custom_layers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_custom_layers::MyCSVLayer, nav2_costmap_2d::Layer)

