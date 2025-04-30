cmake_minimum_required(VERSION 3.8)
project(my_custom_layers)

# === Environment ===


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(obstacle_influence REQUIRED)

# === Build ===
include_directories(include)

add_library(my_custom_layers SHARED
  src/my_csv_layer.cpp
)

# === Installation ===

install(
  TARGETS my_custom_layers
  DESTINATION lib
)


pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  plugin.xml
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# === Ament work ===

ament_target_dependencies(my_custom_layers
  rclcpp
  nav2_costmap_2d
  pluginlib
  obstacle_influence
)
ament_package()
