cmake_minimum_required(VERSION 3.5)
project(prius_msgs)

# https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html#create-custom-definitions
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
   DEPENDENCIES std_msgs
)

ament_package()
