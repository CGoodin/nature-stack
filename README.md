# AVT-341
ROS package with autonomy algorithms for the NATO AVT-341.

## Requirements
Ubuntu 16.04 with [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and a functioning [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) are required to build and run this code. It **may** work with more recent releases of Ubuntu but has not been tested.

## Installation
Clone the repo into your catkin_ws/src directory with the following command.
```bash
$git clone https://github.com/CGoodin/avt_341.git
```
From the top level catkin_ws directory, type
```bash
$catkin_make install
```

To test the installation, type
```bash
$roslaunch avt_341 example.launch
```

##  Troubleshooting
The pakcage requires the ROS PointCloud Library (PCL) interface. If you get errors related to missing pcl header files, then you may need to install pcl_ros on your system.
```bash
$sudo apt-get install ros-kinetic-pcl-ros
```