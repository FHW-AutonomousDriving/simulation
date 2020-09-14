cmake_minimum_required(VERSION 3.0.2)
project(fhw_prius_adas_adapter)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  prius_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/fhw_prius_adas_adapter_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
