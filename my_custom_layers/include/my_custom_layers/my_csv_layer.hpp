#ifndef MY_CUSTOM_LAYERS__MY_CSV_LAYER_HPP_
#define MY_CUSTOM_LAYERS__MY_CSV_LAYER_HPP_

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <utility> 

#include "obstacle_influence/msg/csv_data.hpp"
#include "obstacle_influence/msg/csv_point.hpp"

namespace my_custom_layers
{

class MyCSVLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  MyCSVLayer();
  virtual ~MyCSVLayer();

  virtual void onInitialize() override;

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) override;

  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  virtual bool isClearable() override { return false; }
protected:
  void csvDataCallback(const obstacle_influence::msg::CSVData::SharedPtr msg);

  std::vector<std::tuple<double, double, unsigned char>> csv_cost_data_;

  std::string topic_name_;

  rclcpp::Subscription<obstacle_influence::msg::CSVData>::SharedPtr csv_sub_;

  bool enabled_;
};

}  // namespace my_custom_layers

#endif  // MY_CUSTOM_LAYERS__MY_CSV_LAYER_HPP_

