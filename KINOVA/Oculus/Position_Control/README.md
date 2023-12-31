# KINOVA Remote Manipulation via Positional Control 
Control input: Oculus Quest 2

## Preparation
1. Install related package from [here](https://github.com/AlfaKeNTAvR/relaxed_ik_ros1)
2. Plugin Unity project (download [here](https://drive.google.com/file/d/1XF-1P9C2r3X71h6BOlPYQMhcxCud4nVu/view?usp=sharing))

## Launch the Unity Project
1. Launch Oculus Desktop App
2. Enable Quest Link
3. Run the Oculus Unity Project
4. Click the 'Robotics' tab and select ROS setting, then paste the ROS system IP address
5. Select 'Reset View'
6. Click any buttons on the Oculus control once

## Launch the teleoperation interface
In the first terminal, launch the roscore
```bash
cd kinova_ws/
source ./devel/setup.bash
roscore
```
In the second terminal, launch the kinova
```bash
cd kinova_ws/
source ./devel/setup.bash
roslaunch kortex_driver kortex_driver.launch ip_address:=192.168.1.10 gripper:=robotiq_2f_85 robot_name:=my_gen3
```
In the third terminal, launch the unity-ros connection
```bash
cd kinova_ws/
source ./devel/setup.bash
roslaunch unity_ros endpoint.launch
```
In the fourth terminal, launch the teleoperation interface
```bash
cd kinova_ws/
source ./devel/setup.bash
roslaunch kinova_positional_control oculus_mapping.launch
```
## Launch the graphical user interface
In the fifth terminal, launch the kinova vision
```bash
cd kinova_vision_ws/
source ./devel/setup.bash
roslaunch kinova_vision kinova_vision_color_only.launch
```
In the fifth terminal, launch the graphical user interface
```bash
cd cam_ws/
python3 UI_global_local_views.py
```
