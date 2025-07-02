# Obstacle Influence Costmap Framework

### This module is designed for use with ROS 2 Humble and the Navigation2 stack.
It has been tested in multi-AMR environments using ROS 2 Humble on Ubuntu 22.04 with standard Nav2 navigation behavior.

### This repository provides two ROS 2 packages for cooperative navigation in multi-robot environments:

- **`my_custom_layers`**: A Nav2 plugin layer that integrates real-time costmap data via CSV messages.
- **`obstacle_influence`**: A runtime path-monitoring module that detects blocked regions and shares their influence with other robots.

---

## Package 1: `my_custom_layers`

### Overview

`my_custom_layers` implements `MyCSVLayer`, a plugin for the Nav2 costmap. It dynamically integrates external cost data received as CSV-formatted messages via a ROS 2 topic.

### Features

- Implements `nav2_costmap_2d::Layer`
- Subscribes to `obstacle_influence/msg/CSVData`
- Applies region-specific cost updates efficiently
- Fully compatible with pluginlib

### Build Instructions

```bash
cd ~/your_ros2_ws/src
git clone <this_repository>
cd ~/your_ros2_ws
colcon build --packages-select my_custom_layers
source install/setup.bash
```

### Plugin Configuration

Add the following to your `nav2_params.yaml`:

```yaml
local_costmap:
  plugins: ["static_layer", "inflation_layer", "my_csv_layer"]

my_csv_layer:
  plugin: "my_custom_layers::MyCSVLayer"
  enabled: true
  topic: "/online1"
```

---

## Package 2: `obstacle_influence`

### Overview

`obstacle_influence` monitors robot navigation to detect blocked paths and activates costmap influence in shared regions. It publishes `CSVData` messages, which can be consumed by Nav2 plugins such as `MyCSVLayer`.

### Nodes

- **`blocked_path_publisher`**: Detects path blockage based on robot behavior and activates CSV-based costmap regions.
- **`partition_monitor`**: Observes robot poses and goals to detect inter-partition transitions and trigger influence activation.

### Custom Messages

Located in the `msg/` directory:

- `CSVPoint.msg`: Contains `x`, `y`, and `cost`
- `CSVData.msg`: Contains a `bool enabled` flag and a list of `CSVPoint`

###  Run Instructions

```bash
ros2 run obstacle_influence blocked_path_publisher
ros2 run obstacle_influence partition_monitor
```

Make sure your `csv/` folder contains the appropriate `cost*.csv` files for region-specific influence.

---

## Summary

This system enables **adaptive costmap updates** in response to dynamically detected obstacles by other robots.  


---

## Folder Structure

```
.
├── my_custom_layers/
│   ├── src/my_csv_layer.cpp
│   ├── include/my_csv_layer.hpp
│   ├── plugin.xml
│   ├── CMakeLists.txt
│   └── package.xml
├── obstacle_influence/
│   ├── obstacle_influence/
│   │   ├── blocked_path_publisher.py
│   │   └── partition_monitor.py
│   ├── csv/
│   ├── msg/
│   │   ├── CSVPoint.msg
│   │   └── CSVData.msg
│   ├── setup.py
│   ├── CMakeLists.txt
│   └── package.xml
```

---

## License

This code is made available for academic purposes accompanying a manuscript submission to JIII.

**Note: Unauthorized reproduction, distribution, or modification of this code is strictly prohibited.**

© 2025 Anonymous Authors. All rights reserved.
---

## Contact
If you have questions regarding the paper or this simulation framework, please refer to the official JIII 2025 manuscript submission.
