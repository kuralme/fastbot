# The Fastbot Project

Fastbot is a mobile robot platform developed as part of the Robotics Masterclass course. Built using a commercial robot kit, Fastbot is designed to provide hands-on experience with ROS 2, sensor integration, and autonomous navigation. The robot features differential drive, a suite of sensors (including LiDAR and camera), and is fully compatible with the ROS 2 ecosystem for both simulation and real-world operation.

This repository contains all the ROS 2 packages necessary to operate and extend the Fastbot platform. Essential functionalities, such as subscribing to its lidar and camera topics or publishing velocity commands, are available for controlling the real Fastbot robot.

With the latest addition of the Docker submodule, Fastbot can now be controlled using a containerized ROS 2 environment. This setup enables seamless deployment of SLAM and navigation capabilities on the real robot, making it easier to manage dependencies and run advanced robotics software reliably on the Raspberry Pi 4.

## Prerequisites

- ROS2 Humble (for host setup)
- Docker Engine and Compose (for docker setup)

## Getting Started

Connect to the robot via ssh:

```bash
ssh fastbot@master
```
or enable display to use visual tools such as Rviz2
```bash
ssh fastbot@master -X
```

Clone this repository into your ROS2 workspace `src` directory (create if not exist):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url>
cd fastbot
git submodule update --init --recursive
```

### Host setup

1. Install dependencies:
    ```bash
    # Optional: Use if rosdep not initialized
    sudo rosdep init
    rosdep update
    ```
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
2. Build the workspace:
    ```bash
    colcon build
    ```
3. Source the workspace:
    ```bash
    source install/setup.bash
    ```

#### Launch

To launch the basic robot packages, run command: 

```bash
ros2 launch fastbot_bringup bringup.launch.xml
```
If connected to robot with display is enabled, use this launch instead:
```bash
ros2 launch fastbot_bringup bringup_rviz.launch.xml
```
---

## Docker setup (recommended)

Using docker-compose, the required docker images will be downloaded and started up and ready. The ROS2 network will be accessible from the host computer, Rpi 4.

#### Manual setup

Navigate to the docker directory and setup ROS2 containers.

```bash
cd ~/ros2_ws/src/fastbot/fastbot_ros2_docker/real
sudo chmod +x ros_entrypoint.sh
docker-compose up
```

#### Docker auto-start

The ROS 2 Docker containers can also be automatically started when the robot computer is powered on if set up.

## Test robot sensors and actuation

In second terminal run the following commands to:

- Verify that all the robot systems have been started properly
```bash
ros2 topic list
```
- Subscribe lidar and camera sensor topics
```bash
ros2 topic echo /fastbot/scan
ros2 topic echo /fastbot_camera/image_raw
```
- Move the robot using keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/fastbot/cmd_vel
```