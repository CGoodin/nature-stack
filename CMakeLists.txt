cmake_minimum_required(VERSION 3.0.2)
project(avt_341)

if($ENV{ROS_DISTRO} STREQUAL "noetic")
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

else()
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if($ENV{ROS_DISTRO} STREQUAL "noetic")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_ros
    std_msgs
    tf
  )
else()
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf  
  )
endif()

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
src/node/node_proxy.cpp
)
target_link_libraries(avt_341_perception_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_control_node 
  src/control/avt_341_control_node.cpp 
  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
  src/node/node_proxy.cpp
)

target_link_libraries(avt_341_control_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_local_planner_node 
  src/planning/local/avt_341_local_planner_node.cpp 
  src/planning/local/spline_path.cpp
  src/planning/local/spline_planner.cpp
  src/planning/local/spline_plotter.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_local_planner_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(avt_341_global_path_node 
  src/planning/global/avt_341_global_path_node.cpp 
  src/planning/global/astar.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_global_path_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(avt_341_sim_test_node 
  src/simulation/avt_341_sim_test_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_sim_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(avt_bot_state_publisher_node 
  src/control/avt_bot_state_publisher.cpp 
)
target_link_libraries(avt_bot_state_publisher_node
   ${catkin_LIBRARIES}
)

set(LIB_SOURCES
src/control/pid_controller.cpp 
src/control/pure_pursuit_controller.cpp 
src/perception/elevation_grid.cpp
src/planning/local/spline_path.cpp
src/planning/local/spline_planner.cpp
src/planning/local/spline_plotter.cpp
)

add_library(avt_341 ${LIB_SOURCES})
target_link_libraries(avt_341
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  X11
)

catkin_package(INCLUDE_DIRS include
               LIBRARIES avt_341)

#############
## Install ##
#############

install(TARGETS 
avt_341_perception_node
avt_341_control_node
avt_341_local_planner_node
avt_341_global_path_node
avt_341_sim_test_node
avt_bot_state_publisher_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS avt_341
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
 )