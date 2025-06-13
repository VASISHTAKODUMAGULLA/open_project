



# `Open Project`  ‼️ Work in Progress 👨‍🔧👷

## Group members: 
1) Vasista Kodumagulla
2) Chathuranga Liyanage 
3) Liu Jin yu

## 1. Introduction

This project integrates a Tello drone and a TurtleBot3 robot into a single Gazebo simulation environment, enabling cross-platform coordination (vision-based tello drone and nav2 based navigation) for a unified demonstration. We leverage the TIERS **drone\_racing\_ros2** repository (Galactic + Gazebo 11) for the Tello plugin and models, combined with standard TurtleBot3 assets using gazebo humble. This approach delivers a quick, working demo on ROS 2 Galactic, ideal for a polished simulation while minimizing low-level plugin and middleware porting work .

## 2. Project Scope

* **Core goal:** Spawn both Tello and TurtleBot3 in one world, and implement a drone mission to detect a fire hydrant and share its world coordinates for the TurtleBot to follow using  nav2.
* **Key capabilities:**

  * **Tello**: ROS-driven flight, camera streaming, vision-based object detection, position broadcast.
  * **TurtleBot3**: Gazebo spawn, subscription to drone’s destination location topic, reach the destination
* **Deliverables:**

  1. A combined ROS 2 launch file. It launches both the Tello drone and the turtlebot node. 
  2. Python nodes for the drone to reach the goal, and the TurtleBot reaching the goal using nav2.
  3. Documentation and demo recording of end-to-end operation.

## 3. Installation Steps

1. **System prerequisites** (once per machine):

   ```bash
   sudo apt update
   sudo apt install -y \
     ros-galactic-desktop \
     gazebo11 libgazebo11 libgazebo11-dev \
     ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers \
     ros-galactic-tf2-ros-py ros-galactic-tf2-tools \
     ros-galactic-turtlebot3-gazebo \
     libasio-dev libignition-rendering3
   pip3 install transformations
   ```
2. **Workspace setup**:

   ```bash
   mkdir -p ~/open_project/src
   cd ~/open_project/src
   git clone https://github.com/TIERS/drone_racing_ros2.git
   ```
   
     ```
3. **Copy world file**:

   ```bash
   mkdir -p tello_gazebo/worlds
   cp <your>/open_project_world.sdf tello_gazebo/worlds/
   ```
4. **Build and source**:

   ```bash
   cd ~/drone_race_ws
   source /opt/ros/galactic/setup.bash
   colcon build --symlink-install
   echo "source ~/drone_race_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
5. **Configure Gazebo model path** (add to `~/.bashrc`):

   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
   $(ros2 pkg prefix tello_gazebo)/share/tello_gazebo/models:\
   $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
   ```

## 4. Combined Launch File

Create `tello_gazebo/launch/open_project_demo.launch.py`:


## 5. Mission Workflow & Verification

1. **Launch**:

   ```bash
   source ~/drone_race_ws/install/setup.bash
   source /usr/share/gazebo/setup.sh
   ros2 launch tello_gazebo open_project_demo.launch.py
   ```
2. **Verify streams**:

   ```bash
   ros2 topic echo /drone1/image_raw
   ros2 topic echo /drone1/red_location
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/turtlebot
   ```
3. **Demo**:

   * Drone takes off, searches for red cube, centers, stops, and publishes its world-frame pose.
   * TurtleBot3 subscribe to `/drone1/red_location` and uses nav2 and reach the location. 

## 7. Next Steps & Improvements

* **Multi-drone extension**: spawn additional Tello instances with unique namespaces.
* **Communicating back**: Turtleot3 after reaching the position need to communicate back to the robot. 

## 8. Demonstration Video

Record the simulation as follows:



#### you shall see something like this in the gazebo and also in the rviz tool


https://github.com/user-attachments/assets/94106fde-88e5-4064-8b0f-89feb7ce6953


    

#### References
https://github.com/TIERS/drone_racing_ros2

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/







