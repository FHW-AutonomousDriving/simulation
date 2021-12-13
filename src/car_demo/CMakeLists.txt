cmake_minimum_required(VERSION 3.5)
project(car_demo)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(prius_msgs REQUIRED)
find_package(gazebo 11 REQUIRED) # libgazebo11-dev ros-foxy-gazebo-plugins
find_package(ignition-msgs5 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
ament_target_dependencies(PriusHybridPlugin rclcpp prius_msgs)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

install(TARGETS 
  PriusHybridPlugin
  DESTINATION lib
)
install(DIRECTORY worlds models launch DESTINATION share/${PROJECT_NAME})
ament_package()
