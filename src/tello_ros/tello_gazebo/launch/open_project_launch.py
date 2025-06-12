# open_project_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) Path to your custom SDF world:
    world = Path(os.getenv("HOME")) / "open_project/src/tello_ros/tello_gazebo/worlds/open_project_world.sdf"
    ns1 = 'drone1'
    # ns2 = 'turtlebot'
    # 2) TurtleBot3 SDF lives in turtlebot3_gazebo:
    # tb3_sdf = Path(os.getenv("GAZEBO_MODEL_PATH").split(":")[-1]) / "turtlebot3_burger/model.sdf"
    inject_path = Path(os.getenv("AMENT_PREFIX_PATH").split(":")[0]) / "lib" / "tello_gazebo" / "inject_entity.py"
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    return LaunchDescription([
        # ───────────────────────────────────────────────────────────
        # A) Start the Gazebo server & client with your custom world:
        # ───────────────────────────────────────────────────────────
        ExecuteProcess(
            cmd=["gzserver", str(world), "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=["gzclient"],
            output="screen"
        ),

        # ───────────────────────────────────────────────────────────
        # B) Spawn TurtleBot3 (plugin‐driven spawn_entity.py):
        # ───────────────────────────────────────────────────────────
        # Node(
        #     package="gazebo_ros",
        #     executable="spawn_entity.py",
        #     arguments=[
        #         "-entity", ns2,
        #         "-file", str(tb3_sdf),
        #         "-x", "0.5", "-y", "0", "-z", "0.0"
        #     ],
        #     output="screen"
        # ),

        # Node(
        #     package="tello_gazebo",
        #     executable="inject_entity.py",
        #     arguments=[
        #         "-entity", "tello",
        #         "-file", str(inject_path),
        #         "-x", "1.0", "-y", "0", "-z", "0.0"
        #     ],
        #     output="screen"
        # ),
        # Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
        #      arguments=[urdf_path, '0', '0', '1', '1.57079632679']),


         Node(
             package='tello_gazebo',
             executable='inject_entity.py',
             output='screen',
             arguments=[
                 '-file', str(urdf_path),
                 '-name', ns1,
                 '-x', '0.0',
                 '-y', '0.0',
                 '-z', '1.0',
                 '-yaw', '1.57079632679'
             ]
         ),



        # Node(        # Publish static transforms

        # Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        #      arguments=[urdf_path]),       


        # ───────────────────────────────────────────────────────────
        # C) Launch your two control nodes under proper namespaces:
        # ───────────────────────────────────────────────────────────
        # Node(
        #     package="tb3_pkg",
        #     executable="turtlebot_node",
        #     namespace=ns2,
        #     output="screen"
        # ),
        
        Node(
    	    package='tello_vision',
    	    executable='red_gate_finder',
    	    namespace=ns1,
    	    output='screen'
	),
                # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen',
             namespace=ns1),

        # # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_driver', executable='tello_joy_main', output='screen',
              namespace=ns1),


    ])

