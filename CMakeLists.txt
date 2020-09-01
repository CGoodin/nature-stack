cmake_minimum_required(VERSION 3.0.2)
project(avt_341)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nato_avt_341
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
)
link_directories(
  ${PCL_LIBRARY_DIRS} 
)

add_executable(avt_341_perception_node 
src/perception/avt_341_perception_node.cpp 
src/perception/elevation_grid.cpp
)
target_link_libraries(avt_341_perception_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_control_node 
  src/control/avt_341_control_node.cpp 
  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
)

target_link_libraries(avt_341_control_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_local_planner_node 
  src/planning/local/avt_341_local_planner_node.cpp 
  src/planning/local/spline_path.cpp
  src/planning/local/spline_planner.cpp
)
target_link_libraries(avt_341_local_planner_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_global_path_node 
  src/planning/global/avt_341_global_path_node.cpp 
)
target_link_libraries(avt_341_global_path_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_sim_test_node 
  src/simulation/avt_341_sim_test_node.cpp 
)
target_link_libraries(avt_341_sim_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS 
avt_341_perception_node
avt_341_control_node
avt_341_local_planner_node
avt_341_global_path_node
avt_341_sim_test_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
