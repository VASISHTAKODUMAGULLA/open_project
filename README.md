# `drone_racing_ros2`
## Running a Tello simulation in [Gazebo](http://gazebosim.org/)
## Requirements are Ubuntu 20.04 and ROS2 Galactic
## Contributions: TIERS Lab, University of Turku for the environment setup and the drone injection
## Group members: 
1) Chathuranga Liyanage 
2) Vasista Kodumagulla
3) Liu Jin yu


## Installation
#### Install ROS2 Galactic
    https://docs.ros.org/ with the `ros-galactic-desktop` option.
#### Make sure you have gazebo 
    sudo apt install gazebo11 libgazebo11 libgazebo11-dev
`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing and very simple flight dynamics
* `inject_entity.py` is a script that will read an URDF (ROS) or SDF (Gazebo) file and spawn a model in a running instance of Gazebo
* the built-in camera plugin is used to emulate the Gazebo forward-facing camera
#### Add the following
    sudo apt install libasio-dev
    sudo apt install ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers 
    sudo apt install libignition-rendering3 
    pip3 install transformations


#### Build this package
    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/TIERS/drone_racing_ros2.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build
    
#### Run a teleop simulation

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py
    
You will see a single drone in a blank world.
You can control the drone using the joystick.

If you run into the **No namespace found** error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
#### In a second terminal 
    source install/setup.bash
    cd scripts
    python3 simulation_drone_racing.py
#### In a third terminal     
    source install/setup.bash
    rviz2 rviz
    add the published image topic with the name "gate_image_annotated" to see the gate images being published.

#### you shall see something like this in the gazebo and also in the rviz tool


https://github.com/user-attachments/assets/44e9a054-3b66-4c7c-b5fc-5e183527d523


    

#### Control the drone 
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1






