



# Multi-robot search operation
## This project is a part of the course work of [`Aerial Robotics and Multi robot systems`](https://opas.peppi.utu.fi/en/course/DTEK2084/92053?period=2024-2027) at University of Turku. 
## Introduction

This project demonstrates a ROS 2 Humble-based simulation for coordinated multi-robot search operation using TurtleBot3 and a Tello drone in Gazebo. It integrates a Tello drone and a TurtleBot3 robot within a unified Gazebo simulation environment to demonstrate cross-platform coordination.The TurtleBot3 performs SLAM and navigation, while the drone assists in search operations. We utilize the TIERS **drone_racing_ros2** repository for the Tello simulation plugin originally developed for ROS 2 Galactic and combine it with standard TurtleBot3 assets in ROS 2 Humble. Through careful adaptation, this setup runs successfully on ROS 2 Humble, offering a polished and functional demonstration of multi-robot cooperation in simulation.

## Project Scope

* **Core goal:**
   * Spawn both Tello and TurtleBot3 in one world, and implement a drone mission to detect a fire hydrant and share its world coordinates using ROS 2 topics and TF pose-sharing for the TurtleBot to follow and arrive at the goal using **Nav2** and finally publish a message as Goal Succeded once the Turtle bot reaches the fire hydrant.
* **Key capabilities:**

   * **Tello**: ROS-controlled flight, live camera streaming, vision-based object detection, and global position broadcasting.
   * **TurtleBot3**: Gazebo spawn, subscription to drone’s destination location topic, reach the destination using the prebuilt map and Nav2 navigation.
* **Deliverables:**
  
   * ROS 2 launch file. It launches both the Tello drone and the Turtlebot, including the world in the gazebo environment. 
   * Python node controlling the drone to reach the goal, and then the TurtleBot reaching the goal using nav2.
   * Documentation and demo recording of end-to-end operation.

## Group members: 
1) [Vasista Kodumagulla](https://www.linkedin.com/in/vasista-kodumagulla/) 
2) [Chathuranga Liyanage](https://www.linkedin.com/in/chathuranga-rivimal/) 
3) [Liu Jin yu](https://www.linkedin.com/in/jinyu-liu-b8080b327/)


## Dependencies

Ensure you are using **ROS 2 Humble** on Ubuntu 22.04.
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 

Then follow the steps below.

### Install Required Packages

Open a terminal and install the fllowing. (Dont forget the star '*' after gazebo-)

#### Gazebo
```bash
sudo apt install ros-humble-gazebo-*
sudo apt update
sudo apt install ignition-fortress \
                     ros-humble-ros-gz-sim \
                     ros-humble-ros-gz-bridge \
                     ros-humble-gazebo-ros2-control \
                     ros-humble-gazebo-ros2-control-demos

```
### additional libraries
```bash
sudo apt install libasio-dev
```

#### Cartographer SLAM
```bash
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

####  Navigation2 Stack
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

---

## Setup Instructions

### 1. Create a Workspace and Clone the Project

```bash
mkdir -p ~/open_project_ws/
cd ~/open_project_ws
git clone https://github.com/VASISHTAKODUMAGULLA/open_project.git
```

### 2. Build the Workspace

```bash
colcon build --symlink-install
```
### ❗ Ignore the dev warnings if the colcon build is complete.

### 3. Source the Workspace

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## SLAM Mapping

### 1. Launch Simulation World

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run Cartographer SLAM

Open a new terminal:

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 3. Teleoperate the Robot to Explore

Open a new terminal:

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:**

```
        w
   a    s    d
        x
```

- `w/x`: Move forward/backward
- `a/d`: Turn left/right
- `space` or `s`: Stop

### 4. Save the Map

Open a new terminal:

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---
## Now you may feel free to close all the terminals since the environment is ready for testing :) 
## ❗**Important:** Run this only **after** fully exploring your environment with SLAM Cartographer.
Also you can use the `map.pgm` file and `map.yaml` files if you are just testing my own exact simulation. For simplicity, drop them in your `Home` folder and you are good to go!

## Navigation Simulation 

### 1. Launch Gazebo with Custom World
Open a new terminal in the root work space directory: 

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo open_project.launch.py
```
### ❗Ignore any spwaning errors that you see in the launch terminal if you see the gazebo is launched with both tello and turtlebot and check if the next steps are working, since this is expected in some environments.

### 2. Run Navigation2

In a new terminal:

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

### 3. Estimate Initial Pose in RViz2

- Click **2D Pose Estimate**
- Click on the map where the turtlebot starts and drag the arrow to indicate its direction
- Repeat until the LDS scan overlays correctly on the map

---

## Tello Drone Search Script

### 1. Run the Drone Script

In a new terminal:

```bash
export ROS_DOMAIN_ID=69
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/open_project_ws/open_project/src/scripts
python3 tello_searching_work.py
```

---

## Demonstration Video

#### you shall see something like this in the gazebo and also in the rviz tool


https://github.com/user-attachments/assets/94106fde-88e5-4064-8b0f-89feb7ce6953


## Key Features and what has been achieved as a part of this project: 

- ROS 2 Humble multi-robot simulation.
- SLAM using Cartographer
- Navigation using Nav2 library has been achieved where finally the goal is reached by the turtlebot based on the broadcasted TELLOs global pose.
- Inter-robot TF pose sharing
- Tello drone for search and rescue
- Map saving and reuse for navigation by the turtlebot.

---

## Notes

- Always ensure `ROS_DOMAIN_ID` is consistent across all terminals.
- Adjust the `TURTLEBOT3_MODEL` environment variable as needed (`burger`, `waffle`, or `waffle_pi`).
- Use the saved map YAML path correctly while launching Nav2.
- You can extend the inter-robot pose communication using ROS 2 services or topics (`geometry_msgs/PoseStamped`).

---

## Next Steps & Improvements

* **Multi-drone extension**: spawn additional Tello instances with unique namespaces for an extensive search in vast operation areas.
* **Communicating back**: Turtlebot3 after reaching the position need to communicate back to the robot establishing complete cycle of multirobot communication. 

## Credits

- [TIERS/drone_racing_ros2](https://github.com/TIERS/drone_racing_ros2)
- [ROBOTIS TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [Google Cartographer for ROS](https://google-cartographer-ros.readthedocs.io/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [Gazebo Simulator](https://gazebosim.org/)








