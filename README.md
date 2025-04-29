# -Gazebo-Simulator-for-Cooperative-Costmap

# my_custom_layers

A Custom CSV-Based Costmap Layer Plugin for ROS 2 Navigation2

This repository provides a Nav2 costmap plugin that dynamically updates the costmap based on CSV data published over a ROS 2 topic.

---

## Project Structure

my_custom_layers/
├── CMakeLists.txt # Build instructions
├── package.xml # Package manifest
├── plugin.xml # Plugin description (pluginlib)
├── include/
│ └── my_custom_layers/
│ └── my_csv_layer.hpp # Layer interface definition
└── src/
└── my_csv_layer.cpp # Layer implementation

---

## Key Features

- **Real-Time Topic-Driven Costs**  
  Subscribes to `obstacle_influence::msg::CSVData` to receive (x, y, cost) tuples and apply them to the costmap immediately.

- **Enable/Disable Control**  
  Toggle layer activation via the `my_csv_layer.enabled` parameter or at runtime through incoming messages.

- **Global Coordinate Updates**  
  Applies cost updates in world coordinates, independent of the robot’s current pose.

- **Safe Concurrency**  
  Uses mutex locks to guarantee thread-safe updates of the shared costmap.

- **Pluginlib Integration**  
  Dynamically loaded by Navigation2’s `nav2_costmap_2d` via pluginlib for seamless integration.

---

## Installation

**Prerequisites:**

- ROS 2 Humble  
- Navigation2 (`nav2_costmap_2d`)  
- `pluginlib`  
- `obstacle_influence` message package  

**Steps:**
1. Source ROS 2
source /opt/ros/humble/setup.bash

2. Clone into your workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <your_repo_url> my_custom_layers

3. Build and source
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash



---

## How to Run

1. **Configure Your Costmap YAML**  
   Add the plugin entry under your costmap’s `plugins` section:

costmap:
plugins:
- name: my_csv_layer
type: "my_custom_layers::MyCSVLayer"



2. **Launch Navigation2**  
Pass your costmap parameters file to the Nav2 bringup launch:
ros2 launch nav2_bringup bringup_launch.py
params_file:=<your_costmap_params>.yaml


