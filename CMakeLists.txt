cmake_minimum_required(VERSION 3.5)
project(avt_341)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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

add_executable(avt_341_perception_node
        src/perception/avt_341_perception_node.cpp
        src/perception/elevation_grid.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_perception_node ${dependencies})

add_executable(avt_341_map_publisher_node
        src/perception/avt_341_map_publisher_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_map_publisher_node
        ${dependencies}
        )


add_executable(avt_341_control_node
        src/control/avt_341_control_node.cpp
        src/control/pure_pursuit_controller.cpp
        src/control/pid_controller.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_control_node ${dependencies})

add_executable(avt_341_local_planner_node
        src/planning/local/avt_341_local_planner_node.cpp
        src/planning/local/spline_path.cpp
        src/planning/local/spline_planner.cpp
        src/planning/local/spline_plotter.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
        src/planning/local/rviz_spline_plotter.cpp
        )
ament_target_dependencies(avt_341_local_planner_node ${dependencies} OpenCV)
target_link_libraries(avt_341_local_planner_node
        ${OpenCV_LIBS}
        )
#
add_executable(avt_341_global_path_node
        src/planning/global/avt_341_global_path_node.cpp
        src/planning/global/astar.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
        )
ament_target_dependencies(avt_341_global_path_node ${dependencies} OpenCV)
target_link_libraries(avt_341_global_path_node
        ${OpenCV_LIBS}
        )

add_executable(avt_341_sim_test_node
        src/simulation/avt_341_sim_test_node.cpp
        src/node/node_proxy.cpp
        src/node/clock_publisher.cpp
        src/perception/point_cloud_generator.cpp
        )
ament_target_dependencies(avt_341_sim_test_node ${dependencies})

add_executable(avt_bot_state_publisher_node
        src/control/avt_bot_state_publisher.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_bot_state_publisher_node ${dependencies})

install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        avt_341_perception_node
        avt_341_map_publisher_node
        avt_341_control_node
        avt_341_local_planner_node
        avt_341_global_path_node
        avt_341_sim_test_node
        avt_bot_state_publisher_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
