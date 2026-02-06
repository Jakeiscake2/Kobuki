# ROS 2 on Kobuki TurtleBot

## Overview
TurtleBots are relatively low-cost robot platforms designed to run the open-source Robot Operating System (ROS). The TurtleBot 2 is built on the Kobuki mobile base and was well-supported in ROS 1 Kinetic. Upgrading to ROS 2 Humble (as of February 2024) presents some challenges, which this guide will help you navigate.

## Prerequisites
Ensure that you have the following installed on your system:

- **Ubuntu 24.04** (if you have a different version, you can try other ROS installation)
- **Git** and **colcon** installed

## ROS Kilted Installation:
You can install ROS Kilted following this tutorial.
https://docs.ros.org/en/kilted/Installation/

```bash
#For UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#Ensure the Ubuntu Universe repo is installed 
sudo apt install software-properties-common
sudo add-apt-repository universe

#Install ros-apt-source
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

#OPTIONAL- Install dev tools
sudo apt update && sudo apt install ros-dev-tools

#idk what this is for
sudo apt upgrade

#ROS Desktop install with ROS, RViz, demos, tutorials, etc 
sudo apt install ros-kilted-desktop

#ROS Bare bones 
sudo apt install ros-kilted-ros-base
```
and now you have finished the installation!

If you run the entire command consider some -y commands so you do not need to say yes to every prompt.


## Kobuki Installation Steps

### 1. Install Essential ROS 2 Packages
Run the following command to install the available ROS 2 Kobuki packages via `apt`:
```bash
sudo apt-get install ros-kilted-kobuki-ros-interfaces ros-kilted-kobuki-velocity-smoother
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
'''bash
sudo apt-get install ros-kilted-sophus
'''

**Note:** Compiling this dependency may produce errors related to ISO C++11 macros. You may need to manually apply fixes from relevant commit files.

### uh what 
idk i needed to go this though
```bash
sudo apt install rosdep
sudo rosdep init
rosdep update
```
### 4. Build the Workspace
After cloning the required packages, install dependencies and build the workspace:
```bash
cd ~/kobuki_ros2_ws
rosdep install -i --from-path src --rosdistro kilted -y
colcon build --symlink-install --executor sequential
```

If you get errors, its gonna be here. TBH just chatgpt the error and manually fix it. IDK man, sometimes there's an issue sometimes it is fine.

For some installations, you need to add an operator override

```bash
In file included from /home/ulan/kobuki_ros2_ws/src/ecl_lite/ecl_io/src/lib/../../include/ecl/io/sockets.hpp:28,
                 from /home/ulan/kobuki_ros2_ws/src/ecl_lite/ecl_io/src/lib/../../include/ecl/io/poll.hpp:19,
                 from /home/ulan/kobuki_ros2_ws/src/ecl_lite/ecl_io/src/lib/poll.cpp:13:
/home/ulan/kobuki_ros2_ws/install/ecl_errors/include/ecl/errors/handlers.hpp:73:22: error: ‘virtual void ecl::Error::operator=(const ecl::ErrorFlag&)’ was hidden [-Werror=overloaded-virtual=]
   73 |         virtual void operator=(const ErrorFlag &error) { error_flag = error; }
      |                      ^~~~~~~~
/home/ulan/kobuki_ros2_ws/src/ecl_lite/ecl_io/src/lib/../../include/ecl/io/sockets.hpp:68:21: note:   by ‘ecl::SocketError::operator=’
   68 | class ecl_io_PUBLIC SocketError : public Error
      |  
```
Go to 
```bash
~/kobuki_ros2_ws/src/ecl_lite/ecl_io/include/ecl/io
```
and change 
```bash
...
class ecl_io_PUBLIC SocketError : public Error
{
public:
  /**
   * @brief Configures the error class with the specified error flag.
   *
   * @param flag : the error type.
   */
  SocketError(const ErrorFlag& flag = UnknownError) : Error(flag)
  {}
protected:
  virtual const char* invalidArgErrorString() const
  { return "One of the arguments is invalid (usually a socket descriptor).";}
  ...
```
Into 
```bash
class ecl_io_PUBLIC SocketError : public Error
{
public:
  /**
   * @brief Configures the error class with the specified error flag.
   *
   * @param flag : the error type.
   */
  SocketError(const ErrorFlag& flag = UnknownError) : Error(flag)
  {}
virtual void operator=(const ErrorFlag& error) override {
        Error::operator=(error); // Call the base class operator to assign the error flag
    }
protected:
  virtual const char* invalidArgErrorString() const
  { return "One of the arguments is invalid (usually a socket descriptor).";}
```
Basically just adding 
```bash
virtual void operator=(const ErrorFlag& error) override {
        Error::operator=(error); // Call the base class operator to assign the error flag
    }
```
For my installation, it failed for 
```bash
~/kobuki_ros2_ws/src/ecl_lite/ecl_io/include/ecl/io/sockets.hpp
~/kobuki_ros2_ws/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/errors.hpp
```
and then theres stupid things 
```bash
/opt/ros/kilted/include/message_filters/message_filters/subscriber.h
```
uhh should i ignore errors

After your changed, run this again
```bash
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"
```
Then run this 
```bash
source ./install/setup.bash
```
and you should be good to go.
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
