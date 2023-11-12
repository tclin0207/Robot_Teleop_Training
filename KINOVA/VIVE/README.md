# KINOVA Remote Manipulation via Positional Control 
Control input: HTC VIVE Pro 2

## Install RelaxedIK for KINOVA Gen3 Robotic Arm (Source [here](https://github.com/AlfaKeNTAvR/relaxed_ik_ros1))

1. Clone related packages and build workspace
```bash
cd kinova_ws/src
source ./devel/setup.bash
git clone https://github.com/AlfaKeNTAvR/kinova_pid.git
git clone https://github.com/AlfaKeNTAvR/relaxed_ik_ros1.git
git clone https://github.com/AlfaKeNTAvR/kinova_positional_control
cd ..
catkin_make
```

2. Dependencies
```bash
pip install transformations
sudo pip install readchar
sudo pip install python-fcl
sudo pip install scipy
sudo pip install PyYaml
sudo apt-get install ros-noetic-urdfdom-py
sudo apt-get install ros-noetic-kdl-parser-py
sudo apt-get install ros-noetic-kdl-conversions (optional)
sudo pip install --upgrade numpy
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
rustup install 1.58 
rustup default 1.58
```

3. Initialize RelaxedIK core
```bash
cd kinova_ws/
source ./devel/setup.bash
cd src/relaxed_ik_ros1/
git submodule update --init
cd relaxed_ik_core/
cargo build
cd ..
catkin_make
```
