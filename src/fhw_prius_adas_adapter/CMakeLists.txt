cmake_minimum_required(VERSION 3.5)
project(fhw_prius_adas_adapter)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# shared dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(prius_msgs REQUIRED)

add_executable(node src/fhw_prius_adas_adapter_node.cpp)
ament_target_dependencies(node rclcpp sensor_msgs prius_msgs)

# install all nodes
install(TARGETS 
  node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()