



# `Open Project`  ‼️ Work in Progress

## 1. Introduction

This project demonstrates a ROS 2 Humble-based simulation for coordinated multi-robot behavior using TurtleBot3 and a Tello drone in Gazebo. It integrates a Tello drone and a TurtleBot3 robot within a unified Gazebo simulation environment to demonstrate cross-platform coordination.The TurtleBot3 performs SLAM and navigation, while the drone assists in search operations. We utilize the TIERS **drone_racing_ros2** repository for the Tello simulation plugin originally developed for ROS 2 Galactic and combine it with standard TurtleBot3 assets in ROS 2 Humble. Through careful adaptation, this setup runs successfully on ROS 2 Humble, offering a polished and functional demonstration of multi-robot cooperation in simulation.

## 2. Project Scope

* **Core goal:** Spawn both Tello and TurtleBot3 in one world, and implement a drone mission to detect a fire hydrant and share its world coordinates using ROS 2 topics and TF pose-sharing for the TurtleBot to follow and arrive at the goal using **Nav2**.
* **Key capabilities:**

  * **Tello**: ROS-driven flight, camera streaming, vision-based object detection, position broadcast.
  * **TurtleBot3**: Gazebo spawn, subscription to drone’s destination location topic, reach the destination
* **Deliverables:**

  1. A combined ROS 2 launch file. It launches both the Tello drone and the Turtlebot, including the world in the gazebo environment. 
  2. Python nodes for the drone to reach the goal, and the TurtleBot reaching the goal using nav2.

  3. Documentation and demo recording of end-to-end operation.

## Group members: 
1) Vasista Kodumagulla
2) Chathuranga Liyanage 
3) Liu Jin yu


## Dependencies

Ensure you are using **ROS 2 Humble** on Ubuntu 22.04. Then follow the steps below.

### Install Required Packages

Open a terminal and install the following:

#### Gazebo
```bash
sudo apt install ros-humble-gazebo-*
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

## 🛠 Setup Instructions

### 1. Create a Workspace and Clone the Project

```bash
mkdir -p ~/open_project_ws/
cd ~/open_project_ws
git clone https://github.com/VASISHTAKODUMAGULLA/open_project.git
```

### 2. Build the Workspace

```bash
cd ~/open_project_ws
colcon build --symlink-install
```

### 3. Source the Workspace

```bash
source install/setup.bash
```

---

## SLAM Mapping

### 1. Launch Simulation World

```bash
export ROS_DOMAIN_ID=69
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run Cartographer SLAM

Open a new terminal:

```bash
export ROS_DOMAIN_ID=69
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 3. Teleoperate the Robot to Explore

```bash
export ROS_DOMAIN_ID=69
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

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Navigation Simulation

### 1. Launch Gazebo with Custom World

```bash
export ROS_DOMAIN_ID=69
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo open_project.launch.py
```

### 2. Run Navigation2

In a new terminal:

```bash
export ROS_DOMAIN_ID=69
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

### 3. Estimate Initial Pose in RViz2

- Click **2D Pose Estimate**
- Click on the map where the robot starts and drag the arrow to indicate its direction
- Repeat until the LDS scan overlays correctly on the map

---

## Tello Drone Search Script

### 1. Run the Drone Script

In a new terminal:

```bash
export ROS_DOMAIN_ID=69
source install/setup.bash
cd ~/open_project_ws/src/open_project/scripts
python3 tello_searching_work.py
```

---

## Next Steps & Improvements

* **Multi-drone extension**: spawn additional Tello instances with unique namespaces.
* **Communicating back**: Turtleot3 after reaching the position need to communicate back to the robot. 

## Demonstration Video

Record the simulation as follows:


#### you shall see something like this in the gazebo and also in the rviz tool


https://github.com/user-attachments/assets/94106fde-88e5-4064-8b0f-89feb7ce6953


## Key Features

- ROS 2 Humble multi-robot simulation
- SLAM using Cartographer
- Navigation using Nav2
- Inter-robot TF pose sharing
- Tello drone for search and rescue
- Map saving and reuse for navigation

---

## Notes

- Always ensure `ROS_DOMAIN_ID` is consistent across all terminals.
- Adjust the `TURTLEBOT3_MODEL` environment variable as needed (`burger`, `waffle`, or `waffle_pi`).
- Use the saved map YAML path correctly while launching Nav2.
- You can extend the inter-robot pose communication using ROS 2 services or topics (`geometry_msgs/PoseStamped`).

---

## Credits
  
## [TIERS](https://github.com/TIERS/drone_racing_ros2)  
## [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)  
## [Cartographer](https://google-cartographer-ros.readthedocs.io/) 
## [Navigation2](https://navigation.ros.org/)     
## [Gazebo](https://gazebosim.org/)








