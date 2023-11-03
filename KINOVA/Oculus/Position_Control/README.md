# KINOVA Motion Mapping Teleoperation Interface (Linux only)
Control input: HTC Vive controller

## Preparation
1. Plugin the Vive system (headset and two controllers)
2. Connect the KINOVA via Ethernet (make sure to setup the Internet Protocol Version 4 (TCP/IPv4), 192.168.1.11 for IP address and 255.255.255.0 for subnet mask)
3. Open Steam
4. Open SteamVR
5. Install Vive ROS package
6. Open 5 terminals

## Launch the Vive system
In the first terminal, launch the Vive
```bash
cd vive_ws/
source devel/setup.bash
roslaunch vive_ros vive.launch
```
In the second terminal, launch the workspace calibration
```bash
cd Task_Gen/Data_From_Vive/
python new_world.py
```
In the third terminal, launch the vive frame calibration
```bash
cd Task_Gen/Data_From_Vive/
python vive_transform_correct_frame_new_JACO.py
```
## Launch the KINOVA robotic arm
In the fourth terminal, launch the KINOVA robotic arm (prereques: install the KINOVA ROS package)
```bash
cd kinova_ws/
source devel/setup.bash
roslaunch kortex_driver kortex_driver_v2.launch
```
## Lauch the Vive teleoperation interface
In the fifth terminal, launch the vive interface
```bash
cd kinova_ws/
source devel/setup.bash
cd src/ros_kortex/kortex_examples/src/full_arm/TouchTomorrow2022/
python Vive_OneController.py (Only motion tracking and use side button to pasue and with table avoidance)
```
