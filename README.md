# NATURE-stack
The NATURE (Navigating All Terrains Using Robotic Exploration) autonomy stack is an full stack for autonomous off-road navigation. It features modules for perception, path planning, and vehicle control, with options for Ackermann and skid-steered vehicles.

For more information about the modules and their capabilities and options, check out [the wiki](https://github.com/CGoodin/nature-stack/wiki).

## Requirements
Ubuntu 16.04 with [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and a functioning [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) are required to build and run this code. 

## Installation
Clone the repo into your catkin_ws/src directory with the following command.
```bash
$git clone https://github.com/CGoodin/nature-stack.git nature
```

NATURE can work with ROS or ROS-2, see instructions below.

### ROS-1 Installation
First, copy the installation files for ROS-1 to the correct name.
```bash
$cp CMakeLists_ros1.cmake CMakeLists.txt
$cp package_ros1.xml package.xml
```

Next, from the top level catkin_ws directory, type
```bash
$catkin_make install
```
Or to build only the nature package: 
```bash
$catkin_make --only-pkg-with-deps nature
```

__If user-defined workspace with default install spaces:__ Make sure that ```setup.bash``` has been sourced in either the workspace's ```devel``` or ```install``` folder depending on if ```catkin_make``` or ```catkin_make install``` has been used respectively. Though typically this command is added to ```~/.bashrc``` so that it is called on opening a command prompt instead of being issued manually.  

```bash 
# Can be placed in ~/.bashrc also so does not need to be issued manually
source ~/<path_to_catkin_workspace>/[install|devel]/setup.bash

# Example on my computer (when built with catkin_make): 
source ~/catkin_ws/devel/setup.bash
```

To test the installation, type
```bash
$roslaunch nature example.launch
```

### ROS-2 Installation
First, copy the installation files for ROS-1 to the correct name.
```bash
$cp CMakeLists_ros2.cmake CMakeLists.txt
$cp package_ros2.xml package.xml
```

Next, from the top level catkin_ws directory, type
```bash
$colcon build --symlink-install
```
Or to build only the nature package:
```bash
$colcon build --packages-select nature --symlink install
```

__If user-defined workspace with default install spaces:__ Make sure that ```setup.bash``` has been sourced in either the workspace's ```devel``` or ```install``` folder depending on if ```catkin_make``` or ```catkin_make install``` has been used respectively. Though typically this command is added to ```~/.bashrc``` so that it is called on opening a command prompt instead of being issued manually.  

```bash 
# Can be placed in ~/.bashrc also so does not need to be issued manually
source ~/<path_to_colcon_workspace>/[install|devel]/setup.bash

# Example on my computer (when built with catkin_make): 
source ~/ros2_ws/install/setup.bash
```

To test the installation, type
```bash
$ros2 launch nature example.launch.py
```

##  Troubleshooting
The package requires the ROS PointCloud Library (PCL) interface. If you get errors related to missing pcl header files, then you may need to install pcl_ros on your system.
```bash
$sudo apt-get install ros-kinetic-pcl-ros
```

## Running with MAVS
To run an example simulation with MAVS, first [install and build MAVS](https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator/-/wikis/MavsBuildInstructions).

You will also need the package to interface MAVS to ROS, either the [mavs_ros]() package for ROS 1 or the [mavs-ros2]() package for ROS2. The instructions below are for ROS1, but the process is similar for the mavs-ros2 package.

Clone the MAVS-ROS interface package into your catkin_ws/src directory
```bash
$git clone https://github.com/CGoodin/mavs_ros
```

Then, from the top level catkin_ws directory, type
```bash
$catkin_make install
```
Then, from the "launch" directory of the "nature" folder, run
```bash
$roslaunch nature mavs_example.launch
```

## Funding Acknowledgement
This project is made possible by technical and financial support of the Mississippi State University Center for Advanced Vehicular Systems as well as the Automotive Research Center (ARC) in accordance with Cooperative Agreement W56HZV 14 2 0001 U.S. Army CCDC Ground Vehicle Systems Center (GVSC) Warren, MI.
