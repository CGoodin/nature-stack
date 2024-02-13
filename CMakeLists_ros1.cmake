project(nature)

cmake_minimum_required(VERSION 3.5)

if($ENV{ROS_DISTRO} STREQUAL "noetic")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_ros
    std_msgs
    tf
    message_generation
    #tf2_ros
  )
else()
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    message_generation
    #tf2_ros
  )
endif()

add_definitions(-DROS_1)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nature
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
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
add_definitions(-DROS_1)
add_executable(nature_path_manager_node
  src/planning/global/path_manager_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(nature_path_manager_node
  ${catkin_LIBRARIES}
)

add_executable(nature_gps_to_enu_node
  src/planning/global/gps_to_enu_node.cpp
  src/node/node_proxy.cpp
  src/planning/global/coord_conversions/coord_conversions.cpp
  src/planning/global/coord_conversions/ellipsoid.cpp
  src/planning/global/coord_conversions/matrix.cpp
)
target_link_libraries(nature_gps_to_enu_node
  ${catkin_LIBRARIES}
)

add_executable(nature_gps_spoof_node
  src/planning/global/gps_spoof_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(nature_gps_spoof_node
  ${catkin_LIBRARIES}
)

add_executable(nature_perception_node
src/perception/nature_perception_node.cpp
src/perception/elevation_grid.cpp
src/node/node_proxy.cpp
)
target_link_libraries(nature_perception_node
  ${catkin_LIBRARIES}
)

add_executable(nature_ftte_node 
src/perception/ftte/nature_ftte_node.cpp 
src/perception/ftte/voxel_grid.cpp
src/perception/ftte/vehicle.cpp
src/perception/ftte/matrix.cpp
src/perception/ftte/moreland.cpp
src/node/node_proxy.cpp
)
target_link_libraries(nature_ftte_node
  ${catkin_LIBRARIES} X11 gomp
)

add_executable(nature_map_publisher_node 
src/perception/nature_map_publisher_node.cpp 
src/node/node_proxy.cpp
)
target_link_libraries(nature_map_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(nature_control_node
src/node/node_proxy.cpp  
src/control/nature_control_node.cpp

  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
  
  src/control/tinyfiledialogs.c
)

target_link_libraries(nature_control_node
  ${catkin_LIBRARIES}
)

add_executable(nature_local_planner_node
  src/planning/local/nature_local_planner_node.cpp
  src/planning/local/spline_path.cpp
  src/planning/local/spline_planner.cpp
  src/planning/local/spline_plotter.cpp
  src/planning/local/pf_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
  src/planning/local/rviz_spline_plotter.cpp
)
target_link_libraries(nature_local_planner_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(nature_pf_planner_node
  src/planning/local/nature_pf_planner_node.cpp
  src/planning/local/pf_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
target_link_libraries(nature_pf_planner_node
  ${catkin_LIBRARIES}
  X11
)


add_executable(nature_global_path_node
  src/planning/global/nature_global_path_node.cpp
  src/planning/global/astar.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
target_link_libraries(nature_global_path_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(nature_sim_test_node
  src/simulation/nature_sim_test_node.cpp
  src/node/node_proxy.cpp
  src/node/clock_publisher.cpp
  src/perception/point_cloud_generator.cpp
)
target_link_libraries(nature_sim_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(nature_bot_state_publisher_node
  src/control/nature_bot_state_publisher.cpp
)
target_link_libraries(nature_bot_state_publisher_node
   ${catkin_LIBRARIES}
)

set(LIB_SOURCES
src/control/pid_controller.cpp
src/control/pure_pursuit_controller.cpp
src/perception/elevation_grid.cpp
src/planning/local/spline_path.cpp
src/planning/local/spline_planner.cpp
src/planning/local/spline_plotter.cpp
src/visualization/image_visualizer.cpp
)

add_library(nature ${LIB_SOURCES})
target_link_libraries(nature
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  X11
)

catkin_package(INCLUDE_DIRS include
               LIBRARIES nature)

#############
## Install ##
#############

install(TARGETS
nature_perception_node
nature_ftte_node
nature_map_publisher_node
nature_control_node
nature_local_planner_node
nature_pf_planner_node
nature_global_path_node
nature_sim_test_node
nature_gps_to_enu_node
nature_gps_spoof_node
nature_path_manager_node
nature_bot_state_publisher_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS nature
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
 )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )

install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        FILES_MATCHING PATTERN "*.rviz"
        )
