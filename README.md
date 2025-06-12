



# `Open Project`
## Running a Tello simulation in [Gazebo](http://gazebosim.org/)
## Requirements are Ubuntu 22.04 and ROS2 HUMBLE
## Contributions: TIERS Lab, University of Turku for the environment setup and the drone injection
## Group members: 
1) Chathuranga Liyanage 
2) Vasista Kodumagulla
3) Liu Jin yu


# Open Project Documentation

## 1. Introduction

This project integrates a Tello drone and a TurtleBot3 robot into a single Gazebo simulation environment, enabling cross-platform coordination (vision-based tello drone and nav2 based navigation) for a unified demonstration. We leverage the TIERS **drone\_racing\_ros2** repository (Galactic + Gazebo 11) for the Tello plugin and models, combined with standard TurtleBot3 assets. This approach delivers a quick, working demo on ROS 2 Galactic, ideal for a polished simulation video while minimizing low-level plugin and middleware porting work .

## 2. Project Scope

* **Core goal:** Spawn both Tello and TurtleBot3 in one world, teleoperate each, and implement a drone mission to detect a fire hydrant and share its world coordinates for the TurtleBot to consume follow using  nav2.
* **Key capabilities:**

  * **Tello**: ROS-driven flight, camera streaming, vision-based object detection, position broadcast.
  * **TurtleBot3**: Gazebo spawn, subscription to drone’s destination location topic.
* **Deliverables:**

  1. A combined ROS 2 launch file. It launches both the tello drone and the turtlebot node. 
  2. Python nodes for drone to reach to the goal and TurtleBot reaching the goal and map of the world.
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
   mkdir -p ~/drone_race_ws/src
   cd ~/drone_race_ws/src
   git clone https://github.com/TIERS/drone_racing_ros2.git
   ros2 pkg create --build-type ament_python tello_pkg
   ros2 pkg create --build-type ament_python tb3_pkg
   ```
3. **Add custom Python nodes**:

   * Copy your `red_gate_finder.py` into `tello_pkg/tello_pkg/`
   * Copy `turtlebot_node.py` into `tb3_pkg/tb3_pkg/`
   * Update each `setup.py` with proper `entry_points` for console scripts.
   * Add dependencies to `package.xml`:

     ```xml
     <depend>rclpy</depend>
     <depend>geometry_msgs</depend>
     <depend>sensor_msgs</depend>
     <depend>cv_bridge</depend>
     <depend>tf2_ros</depend>
     <depend>tello_msgs</depend>
     ```
4. **Copy world file**:

   ```bash
   mkdir -p tello_gazebo/worlds
   cp <your>/open_project_world.sdf tello_gazebo/worlds/
   ```
5. **Build and source**:

   ```bash
   cd ~/drone_race_ws
   source /opt/ros/galactic/setup.bash
   colcon build --symlink-install
   echo "source ~/drone_race_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
6. **Configure Gazebo model path** (add to `~/.bashrc`):

   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
   $(ros2 pkg prefix tello_gazebo)/share/tello_gazebo/models:\
   $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
   ```

## 4. Combined Launch File

Create `tello_gazebo/launch/open_project_demo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from pathlib import Path
import os

def generate_launch_description():
    home = Path(os.getenv("HOME"))
    world = home / "drone_race_ws/src/tello_gazebo/worlds/open_project_world.sdf"
    tb3_sdf = Path(os.getenv("GAZEBO_MODEL_PATH").split(":")[-1]) / "turtlebot3_burger/model.sdf"
    urdf = Path(os.getenv("AMENT_PREFIX_PATH").split(":")[0]) / "share/tello_description/urdf/tello.urdf"

    return LaunchDescription([
        # Gazebo
        ExecuteProcess(cmd=["gzserver", str(world), "-s", "libgazebo_ros_factory.so"], output="screen"),
        ExecuteProcess(cmd=["gzclient"], output="screen"),

        # Spawn Tello via plugin
        Node(package="tello_gazebo", executable="inject_entity.py",
             arguments=["-file", str(urdf), "-name", "drone1", "-x", "0", "-y", "0", "-z", "1", "-yaw", "0"],
             output="screen"),

        # Spawn TurtleBot3
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "turtlebot", "-file", str(tb3_sdf), "-x", "0.5", "-y", "0", "-z", "0.0"],
             output="screen"),

        # High-level nodes
        Node(package="tello_pkg", executable="red_gate_finder", namespace="drone1", output="screen"),
        Node(package="tb3_pkg", executable="turtlebot_node", namespace="turtlebot", output="screen"),
    ])
```

## 5. Common Issues & Solutions

| Symptom                                    | Cause                                          | Fix                                                           |
| ------------------------------------------ | ---------------------------------------------- | ------------------------------------------------------------- |
| `TelloPlugin: libopencv_core.so not found` | Missing OpenCV dev libs                        | `sudo apt install libopencv-dev`                              |
| Black camera image                         | Missing `source /usr/share/gazebo/setup.sh`     | Add to `~/.bashrc` or run manually                            |
| TurtleBot3 invisible                       | Wrong `GAZEBO_MODEL_PATH`                      | Verify model path points to `turtlebot3_gazebo/models`        |
| QoS incompatibility warnings               | Subscriber QoS doesn’t match publisher default | Use `ReliabilityPolicy.BEST_EFFORT` for camera topics in code |

## 6. Mission Workflow & Verification

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

* **Port to ROS 2 Humble + Gazebo Garden** via `ros_gz_sim` and `ros_gz_bridge` to future-proof middleware and practice bridge architecture.
* **Autonomous TurtleBot navigation** using Nav2, driven by the drone’s published gate location.
* **UI dashboard** combining camera feed, robot pose, and mission status.
* **Multi-drone extension**: spawn additional Tello instances with unique namespaces.
* **Communicating back**: Turtleot3 after reaching the position need to communicate back to the robot. 

## 8. Demonstration Video

Record the simulation as follows:



#### you shall see something like this in the gazebo and also in the rviz tool


https://github.com/user-attachments/assets/94106fde-88e5-4064-8b0f-89feb7ce6953


    

#### Control the drone 
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1






