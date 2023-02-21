project(nature)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)

if (WIN32 OR WIN64)
set (link_libs
${OpenCV_LIBS}
)
else()
 find_package(X11 REQUIRED)
set (link_libs
${OpenCV_LIBS}
X11
)
endif()

set(dependencies
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        std_msgs
        tf2_ros
        )

###########
## Build ##
###########
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(nature_path_manager_node
   src/planning/global/path_manager_node.cpp
   src/node/node_proxy.cpp
 )
 ament_target_dependencies(nature_path_manager_node ${dependencies})

add_executable(nature_gps_to_enu_node
  src/planning/global/gps_to_enu_node.cpp
  src/node/node_proxy.cpp
  src/planning/global/coord_conversions/coord_conversions.cpp
  src/planning/global/coord_conversions/ellipsoid.cpp
  src/planning/global/coord_conversions/matrix.cpp
)
ament_target_dependencies(nature_gps_to_enu_node ${dependencies})

add_executable(nature_gps_spoof_node
  src/planning/global/gps_spoof_node.cpp
  src/node/node_proxy.cpp
)
ament_target_dependencies(nature_gps_spoof_node ${dependencies})

add_executable(nature_perception_node 
src/perception/nature_perception_node.cpp 
src/perception/elevation_grid.cpp
src/node/node_proxy.cpp
)
ament_target_dependencies(nature_perception_node ${dependencies})

add_executable(nature_map_publisher_node 
src/perception/nature_map_publisher_node.cpp 
src/node/node_proxy.cpp
)
ament_target_dependencies(nature_map_publisher_node ${dependencies})

add_executable(nature_control_node 
  src/control/nature_control_node.cpp 
  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
  src/control/tinyfiledialogs.c
  src/node/node_proxy.cpp
)
ament_target_dependencies(nature_control_node ${dependencies})

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
ament_target_dependencies(nature_local_planner_node ${dependencies})
target_link_libraries(nature_local_planner_node ${link_libs} )


add_executable(nature_pf_planner_node 
  src/planning/local/nature_pf_planner_node.cpp 
  src/planning/local/pf_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
ament_target_dependencies(nature_pf_planner_node ${dependencies})
target_link_libraries(nature_pf_planner_node ${link_libs} )

add_executable(nature_global_path_node 
  src/planning/global/nature_global_path_node.cpp 
  src/planning/global/astar.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
ament_target_dependencies(nature_global_path_node ${dependencies})
target_link_libraries(nature_global_path_node ${link_libs} )

add_executable(nature_sim_test_node 
  src/simulation/nature_sim_test_node.cpp
  src/node/node_proxy.cpp
  src/node/clock_publisher.cpp
  src/perception/point_cloud_generator.cpp
)
ament_target_dependencies(nature_sim_test_node ${dependencies})

add_executable(nature_bot_state_publisher_node 
  src/control/nature_bot_state_publisher.cpp 
)
ament_target_dependencies(nature_bot_state_publisher_node ${dependencies})



# add_executable(avt_341_perception_node
#         src/perception/avt_341_perception_node.cpp
#         src/perception/elevation_grid.cpp
#         src/node/node_proxy.cpp
#         )
# ament_target_dependencies(avt_341_perception_node ${dependencies})

# add_executable(avt_341_map_publisher_node
#         src/perception/avt_341_map_publisher_node.cpp
#         src/node/node_proxy.cpp
#         )
# ament_target_dependencies(avt_341_map_publisher_node
#         ${dependencies}
#         )


# add_executable(avt_341_control_node
#         src/control/avt_341_control_node.cpp
#         src/control/pure_pursuit_controller.cpp
#         src/control/pid_controller.cpp
#         src/node/node_proxy.cpp
#         )
# ament_target_dependencies(avt_341_control_node ${dependencies})

# add_executable(avt_341_speed_control_node
#         src/control/avt_341_speed_control_node.cpp
#         src/control/pid_controller.cpp
#         src/node/node_proxy.cpp
#         )
# ament_target_dependencies(avt_341_speed_control_node ${dependencies})

# add_executable(speed_control_test_node
#         src/control/speed_control_test_node.cpp
#         src/node/node_proxy.cpp
#         )
# ament_target_dependencies(speed_control_test_node ${dependencies})

# add_executable(avt_341_local_planner_node
#         src/planning/local/avt_341_local_planner_node.cpp
#         src/planning/local/spline_path.cpp
#         src/planning/local/spline_planner.cpp
#         src/planning/local/spline_plotter.cpp
#         src/node/node_proxy.cpp
#         src/visualization/image_visualizer.cpp
#         src/planning/local/rviz_spline_plotter.cpp
#         )
# ament_target_dependencies(avt_341_local_planner_node ${dependencies} OpenCV)
# target_link_libraries(avt_341_local_planner_node
#         ${link_libs}
#         )

# add_executable(avt_341_pf_planner_node 
#         src/planning/local/avt_341_pf_planner_node.cpp 
#         src/planning/local/pf_planner.cpp
#         src/node/node_proxy.cpp
#         src/visualization/image_visualizer.cpp
#       )
# ament_target_dependencies(avt_341_pf_planner_node ${dependencies} )
# target_link_libraries(avt_341_pf_planner_node
# ${link_libs}
#       )

# add_executable(avt_341_global_path_node
#         src/planning/global/avt_341_global_path_node.cpp
#         src/planning/global/astar.cpp
#         src/node/node_proxy.cpp
#         src/visualization/image_visualizer.cpp
#         )
# ament_target_dependencies(avt_341_global_path_node ${dependencies} OpenCV)
# target_link_libraries(avt_341_global_path_node
# ${link_libs}
#         )

# add_executable(avt_341_sim_test_node
#         src/simulation/avt_341_sim_test_node.cpp
#         src/node/node_proxy.cpp
#         src/node/clock_publisher.cpp
#         src/perception/point_cloud_generator.cpp
#         )
# ament_target_dependencies(avt_341_sim_test_node ${dependencies})

# add_executable(avt_bot_state_publisher_node
#         src/control/avt_bot_state_publisher.cpp
#         src/node/node_proxy.cpp
#         )
# ament_target_dependencies(avt_bot_state_publisher_node ${dependencies})

# if (WIN32 OR WIN64)
# # this should point to the installation location of MATLAB Runtime
# find_package(Matlab)

#  if (Matlab_FOUND)
#          set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\v912\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
#          include_directories(
#                  include
#                  ${OpenCV_INCLUDE_DIRS}
#                  ${Matlab_INCLUDE_DIRS}
#          )
#          add_executable(uab_perception_node
#                  src/perception/uab_perception_node.cpp
#                  src/node/node_proxy.cpp
#          )
#          ament_target_dependencies(uab_perception_node ${dependencies})
#          target_link_libraries(uab_perception_node
#                  ${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.lib
#                  ${Matlab_MCLMCRRT_LIB}
#          )
#          install(FILES
#                  ${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.dll
#                  DESTINATION lib/${PROJECT_NAME})
#          install(TARGETS
#                  uab_perception_node
#                  EXPORT export_${PROJECT_NAME}
#                  DESTINATION lib/${PROJECT_NAME})
#  endif()

# endif()

install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
nature_perception_node
nature_map_publisher_node
nature_control_node
nature_local_planner_node
nature_pf_planner_node
nature_global_path_node
nature_sim_test_node
#nature_gps_to_enu_node
#nature_gps_spoof_node
#nature_path_manager_node
nature_bot_state_publisher_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
