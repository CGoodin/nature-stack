# NATURE-stack
The NATURE (Navigating All Terrains Using Robotic Exploration) autonomy stack is an full stack for autonomous off-road navigation. It features modules for perception, path planning, and vehicle control, with options for Ackermann and and skid-steered vehicles.

For more information about the modules and their capabilities and options, check out [the wiki](https://github.com/CGoodin/nature-stack/wiki).

## Requirements
Ubuntu 16.04 with [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and a functioning [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) are required to build and run this code. It **may** work with more recent releases of Ubuntu but has not been tested.

## Installation
Clone the repo into your catkin_ws/src directory with the following command.
```bash
$git clone https://github.com/CGoodin/nature-stack.git
```
From the top level catkin_ws directory, type
```bash
$catkin_make install

# Or to build only this package: 
$catkin_make --only-pkg-with-deps avt_341
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
$roslaunch avt_341 example.launch
```

##  Troubleshooting
The package requires the ROS PointCloud Library (PCL) interface. If you get errors related to missing pcl header files, then you may need to install pcl_ros on your system.
```bash
$sudo apt-get install ros-kinetic-pcl-ros
```

## Running with MAVS
To run an example simulation with MAVS, first [install and build MAVS](https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator/-/wikis/MavsBuildInstructions).

Next, clone the example MAVS simulation package.
```bash
$git clone https://github.com/CGoodin/mavs_avt_example.git
```

In order to build the repo, you will need to modify the CMakeLists.txt file in the mavs_avt_example to find your MAVS installation. In lines 8-9, change the following lines 
```
SET(mavs_INCLUDE_DIR  "/home/msucavs/mavs/src/")
SET(mavs_LIB_DIR  "/home/msucavs/mavs/build/lib")
```
to match the path to the MAVS "src" and "lib" directories on your system. Then, from the top level catkin_ws directory, type
```bash
$catkin_make install
```

To use the MAVS example with the AVT-341 autonomy, uncomment lines 58-62 of "example.launch" and comment out line 52. Then, run the example as before.
```bash
$roslaunch avt_341 example.launch
```

This project is made possible by technical and financial support of the Mississippi State University Center for Advanced Vehicular Systems as well as the Automotive Research Center (ARC) in accordance with Cooperative Agreement W56HZV 14 2 0001 U.S. Army CCDC Ground Vehicle Systems Center (GVSC) Warren, MI.
