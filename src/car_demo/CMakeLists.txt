cmake_minimum_required(VERSION 3.5)
project(car_demo)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(prius_msgs REQUIRED)
find_package(gazebo 11 REQUIRED) # libgazebo11-dev
find_package(ignition-msgs5 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
ament_target_dependencies(PriusHybridPlugin rclcpp prius_msgs)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

#~ install(TARGETS 
  #~ PriusHybridPlugin
  #~ DESTINATION lib/${PROJECT_NAME}
#~ )
install(DIRECTORY worlds models DESTINATION share/${PROJECT_NAME})
ament_package()

#~ set (PRIUSCUP_MAJOR_VERSION 0)
#~ set (PRIUSCUP_MINOR_VERSION 1)
#~ set (PRIUSCUP_PATCH_VERSION 0)
#~ set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})
#~ catkin_package(
 #~ # INCLUDE_DIRS include
 #~ # LIBRARIES gazebo_radar_plugin
 #~ CATKIN_DEPENDS gazebo_ros prius_msgs
#~ #  DEPENDS system_lib
#~ )
#~ catkin_install_python(PROGRAMS nodes/joystick_translator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#~ install(TARGETS PriusHybridPlugin
  #~ ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #~ LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #~ RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#~ )
