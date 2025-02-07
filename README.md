# ROS 2 on Kobuki TurtleBot

## Overview
TurtleBots are relatively low-cost robot platforms designed to run the open-source Robot Operating System (ROS). The TurtleBot 2 is built on the Kobuki mobile base and was well-supported in ROS 1 Kinetic. Upgrading to ROS 2 Humble (as of February 2024) presents some challenges, which this guide will help you navigate.

## Prerequisites
Ensure that you have the following installed on your system:

- **Ubuntu 22.04** (or the appropriate OS for ROS 2 Humble)
- **ROS 2 Humble** installed and sourced
- **Git** and **colcon** installed

## Installation Steps

### 1. Install Essential ROS 2 Packages
Run the following command to install the available ROS 2 Kobuki packages via `apt`:
```bash
sudo apt-get install ros-humble-kobuki-ros-interfaces ros-humble-kobuki-velocity-smoother
```

### 2. Create a Workspace
Open a terminal and create a workspace for the Kobuki TurtleBot ROS 2 setup:
```bash
mkdir -p ~/kobuki_ros2_ws/src
cd ~/kobuki_ros2_ws/src
```
Clone the necessary repositories:
```bash
git clone https://github.com/kobuki-base/kobuki_core.git
git clone https://github.com/kobuki-base/kobuki_ros.git
git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git
git clone https://github.com/kobuki-base/cmd_vel_mux.git
git clone https://github.com/stonier/ecl_core.git
git clone https://github.com/stonier/ecl_lite.git
```

### 3. Compile Additional Packages

Install the `Sophus` library package:
```bash
sudo apt-get install ros-humble-sophus
```

**Note:** Compiling this dependency may produce errors related to ISO C++11 macros. You may need to manually apply fixes from relevant commit files.

### 4. Build the Workspace
After cloning the required packages, install dependencies and build the workspace:
```bash
cd ~/kobuki_ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install --executor sequential
```


## Setting Up USB Connection

### Important: USB Permissions
To ensure proper access to the Kobuki base, add your user to the `dialout` group:
```bash
sudo usermod -aG dialout $USER
reboot
```
This step prevents permission issues when connecting to the Kobuki base.

Follow the official Kobuki documentation to:
- Update `udev` rules for the USB connection.
- Check version information.
- Run the `kobuki-simple-keyop` test node to confirm robot movement.

## Testing the Installation
To verify that the Kobuki driver is installed correctly, launch the test node:
```bash
ros2 launch kobuki_node kobuki_node-launch.py
```
If everything is set up properly, you should hear the startup tune, indicating that the robot is ready.

## ROS 2 Teleoperation
Teleoperation allows remote control of the robot. Since ROS 2 teleop nodes publish to `/cmd_vel`, but Kobuki expects input from `/commands/velocity`, a remapping is required.

###  Keyboard Teleoperation
To control the robot using a keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=commands/velocity
```
Use the keyboard to navigate the robot.

