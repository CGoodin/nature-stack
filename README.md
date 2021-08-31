# AVT-341 (ROS 2 Version)
ROS2 package with autonomy algorithms for the NATO AVT-341.

## Requirements
Windows or Linux-based OS with [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/) or greater.

## Installation
__Get ros2 branch with either:__
 - Method 1: Clone into new folder
```bash
cd <path_to_workspace>
# Example: cd C:\Users\Stefan\source\ros_ws

git clone -b ros2 https://github.com/CGoodin/avt_341.git
```

 - Method 2: I am already on master (ROS1) branch and want to switch to ROS2: 
```bash
git fetch 
git checkout ros2
```

__Source ros2 instalation:__
```bash
call <path_to_ros2_installation>\local_setup.bat
# Example: call C:\dev\ros2_foxy\local_setup.bat 
```
__Build package:__
```bash
# On Windows
colcon build --merge-install --event-handlers console_cohesion+ --packages-select avt_341
 
# On Linux
colcon build --event-handlers console_cohesion+ --packages-select avt_341 
```
__To test the installation, source the build repository and launch:__
```bash
call .\install\local_setup.bat
ros2 launch avt_341 example.launch.py
```
__RVIZ Visualization:__
- The ROS2 branch uses RVIZ visualization by default instead of X11 images in the ROS1 branch
- Launch RVIZ with the required configuration file to visualize sensor messages of the control stack with:
```bash
ros2 run rviz2 rviz2 -d "<path_to_avt_341_repo>\rviz\avt_341_ros2.rviz"
# Example: ros2 run rviz2 rviz2 -d "C:\Users\Stefan\source\ros_ws\src\avt_341\rviz\avt_341_ros2.rviz" 
```
