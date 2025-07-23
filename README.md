# The Fastbot Project

Fastbot is a mobile robot platform developed as part of the Robotics Masterclass course. Built using a commercial robot kit, Fastbot is designed to provide hands-on experience with ROS 2, sensor integration, and autonomous navigation. The robot features differential drive, a suite of sensors (including LiDAR and camera), and is fully compatible with the ROS 2 ecosystem for both simulation and real-world operation.

This repository contains all the ROS 2 packages necessary to operate and extend the Fastbot platform. Essential functionalities, such as subscribing to its lidar and camera topics or publishing velocity commands, are available for controlling the real Fastbot robot.

## Installation Instructions

**Prerequisites:**

Connect to the robot via ssh:

```bash
ssh fastbot@master
```
or enable display to use visual tools such as Rviz2
```bash
ssh fastbot@master -X
```
The password is: `fastbot-ege`

**Steps:**
1. Clone this repository into your ROS2 workspace `src` directory:
    ```bash
    cd ~/ros2_ws/src
    git clone <this-repo-url>
    cd fastbot
    git submodule update --init --recursive
    ```
2. Install dependencies:
    ```bash
    # Optional: Use if rosdep not initialized
    sudo rosdep init
    rosdep update
    ```
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:
    ```bash
    colcon build
    ```
4. Source the workspace:
    ```bash
    source install/setup.bash
    ```
---

## Getting Started

To launch the simulation use command given below. 

```bash
ros2 launch fastbot_bringup bringup.launch.xml
```
If connected to robot with display is enabled, use this launch instead:
```bash
ros2 launch fastbot_bringup bringup_rviz.launch.xml
```

In a second terminal connect to robot and check topic outputs:

```bash
ros2 topic echo /fastbot/scan
ros2 topic echo /fastbot_camera/image_raw
```

## Test robot actuation
On second terminal, run the following command to move the robot using keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/fastbot/cmd_vel
```

