cmake_minimum_required(VERSION 3.8)
project(obstacle_influence)

# === Environment ===


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclpy REQUIRED)


find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CSVPoint.msg"
  "msg/CSVData.msg"
)




install(
  PROGRAMS
    obstacle_influence/blocked_path_publisher.py
    obstacle_influence/partition_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
