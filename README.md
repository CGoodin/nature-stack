# AVT-341 (ROS 2 Version)
ROS2 package with autonomy algorithms for the NATO AVT-341.

## Requirements
Windows or Linux-based OS with [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/) or greater.

## Installation
Clone the repo into your ros workspace directory with the following command:
```bash
cd <path_to_workspace>
# Example: cd C:\Users\Stefan\source\ros_ws

git clone -b ros2 https://github.com/CGoodin/avt_341.git
```
Source ros2 instalation:
```bash
call <path_to_ros2_installation>\local_setup.bat
# Example: call C:\dev\ros2_foxy\local_setup.bat 
```
Build package:
```bash
# On Windows
colcon build --merge-install --event-handlers console_cohesion+ --packages-select avt_341
 
# On Linux
colcon build --event-handlers console_cohesion+ --packages-select avt_341 
```
To test the installation, source the build repository and launch:
```bash
call .\install\local_setup.bat
ros2 launch avt_341 example.launch.py
```
RVIZ Visualization:
```bash
ros2 run rviz2 rviz2 -d "<path_to_avt_341_repo>\rviz\avt_341.rviz"
# Example: ros2 run rviz2 rviz2 -d "C:\Users\Stefan\source\ros_ws\src\avt_341\rviz\avt_341.rviz" 
```