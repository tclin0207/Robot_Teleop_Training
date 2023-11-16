# KINOVA Remote Manipulation via Positional Control 
Control input: HTC VIVE Pro 2

## Preparation
1. Build the workspace (download vive workspace from [here]())
2. Plugin Unity project (download from [here](https://drive.google.com/file/d/1XF-1P9C2r3X71h6BOlPYQMhcxCud4nVu/view?usp=sharing))
3. Install Gstreamer (source:)

## Launch the teleoperation interface
1. Open and play the unity project

In the first terminal, launch the roscore
```bash
cd vive_ws/
source ./devel/setup.bash
roscore
```
In the second terminal, launch the kinova
```bash
cd vive_ws/
source ./devel/setup.bash
roslaunch kortex_driver kortex_driver.launch ip_address:=192.168.1.10 gripper:=robotiq_2f_85 robot_name:=my_gen3
```
In the third terminal, launch the unity-ros connection
```bash
cd unity_ros_vive_ws/
python3 socket_connection.py 
```
In the fourth terminal, launch the teleoperation interface
```bash
cd vive_ws/
source ./devel/setup.bash
roslaunch kinova_positional_control vive_kinova_control.launch
```
## Launch the graphical user interface
TBD
