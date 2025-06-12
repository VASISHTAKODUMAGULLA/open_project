#!/usr/bin/env python3

# ──────────────────────────────────────────────────────────────────────────────  
# ──────────────────────────────────────────────────────────────────────────────

#command line usage:
#python3 simulation_drone_racing.py            # ➔ real-world mode
#python3 simulation_drone_racing.py --sim      # ➔ simulation mode

# ──────────────────────────────────────────────────────────────────────────────

import sys
import time
import rclpy
import cv2
import numpy as np
import math


from rclpy.action import ActionClient
from rclpy.node        import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from rclpy.qos         import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg   import Image
from geometry_msgs.msg import Twist
from tello_msgs.srv    import TelloAction
from cv_bridge         import CvBridge
import cv2.aruco as aruco
import tf2_ros
from tf2_geometry_msgs import PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry



class DroneRacing(Node):
    def __init__(self):
        super().__init__('shape_flyer')
        self.bridge = CvBridge()
        # self.simulation = simulation
        # ─── FSM state ───
        self.state        = 'SEARCH'   # start by climbing
        self.latest       = None      # last detected object (gate or stop)
        self.frame        = None

        # ─── ALIGN gains + deadbands ───

        self.Kp_x, self.Kp_y = 0.1, 0.1                 #Kp_x, Kp_y: proportional gains for yaw (horizontal) and z (vertical) control in ALIGN.
        self.dead_x, self.dead_y = 0.1, 0.1



        # ─── timing & counters ───

        self.search_yaw_speed   = 0.3
        self.centered_frames    = 3
        self.centered_count     = 0
        self.forward_duration   = 0.25
        self.forward_start      = None
        self.stop_forward_dur   = 1.0


        self.image = None



        #red HSV range
        self.stop_low  = np.array((0,  50,  0))        #red_low: lower bound of green HSV range.
        self.stop_high = np.array((180, 255, 50))
        self.l1 = (0, 50, 0)
        self.h1 = (10, 255, 80)
        self.l2 = (170, 50, 0)
        self.h2 = (180, 255, 80)
        self.area = 0


        self.blind_start      = None

        # ─── ROS interfaces ───
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            Image, '/drone1/image_raw', self.image_cb, qos_profile=camera_qos
        )


        #Action client for navigation of turtlebot 
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.timer = self.create_timer(1.0, self.send_goal)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)        
#        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.drone_pose = None
        self.create_subscription(Odometry, '/drone1/odom', self.odom_cb, 10)

        self.land_pub = self.create_publisher(PoseStamped, '/drone1/landing_location', 10)

        self.annotated_image_pub = self.create_publisher(Image, '/drone1/gate_image_annotated', 10)
        self.cmd_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.cli     = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /drone1/tello_action…')
        self.takeoff()
        # ─── control loop @20 Hz ───
        self.create_timer(1/20, self.control_loop)

    def odom_cb(self, msg: Odometry):
        self.drone_pose = msg.pose.pose

    
    def takeoff(self):
        req = TelloAction.Request(); req.cmd='takeoff'
        self.cli.call_async(req)
        time.sleep(5.0)
        cmd_publish = Twist()
        cmd_publish.linear.z = -0.2
        self.cmd_pub.publish(cmd_publish)
        time.sleep(0.5)
        cmd_publish.linear.z = 0.0
        self.cmd_pub.publish(cmd_publish)


    def land(self):
        req = TelloAction.Request(); req.cmd='land'
        self.cli.call_async(req)


    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.image = frame.copy()
        self.annotated_image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, 'bgr8'))


        stop = self.detect_hydrant(frame)
        
        if stop:
            print("HYDRANT detected")
            self.latest = stop
            self.state = 'ALIGN_HYDRANT'
            return

        # Nothing detected
        self.latest = None





    def send_goal(self,pose_stamped: PoseStamped):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        x = pose_stamped.pose.position.x + 2.0   #relative robots positionining
        y = pose_stamped.pose.position.y + 0.5
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.w = 1.0
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: Goal completed!')




    
    def detect_hydrant(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Optional: threshold to improve text clarity
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)   
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l1,h1 = np.array(self.l1), np.array(self.h1)
        l2,h2 = np.array(self.l2), np.array(self.h2)
        m = cv2.bitwise_or(cv2.inRange(hsv,l1,h1), cv2.inRange(hsv,l2,h2))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k, iterations=1)

        cnts,_ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        # for c in cnts:
        c = max(cnts, key=cv2.contourArea)
        A = cv2.contourArea(c)
        self.area = A
        # print("Red:",A)

        if A<500: 
            # continue
            return None
        if A> 250000:
            # self.land(); rclpy.shutdown()
            print("Goal found!")
            cmd=Twist(); cmd.linear.x=0.0
            self.cmd_pub.publish(cmd)

            rclpy.spin_once(self, timeout_sec=1.0)

            try:    
                
                if self.drone_pose is not None:
                    ps = PoseStamped()
                    ps.header.frame_id = 'odom'
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.pose = self.drone_pose

                    self.land_pub.publish(ps)
                    self.get_logger().info(
                        f"Goal pose published: {ps.pose.position.x}, {ps.pose.position.y}, {ps.pose.position.z}"
                    )
                    self.send_goal(ps)
                else:
                    self.get_logger().warn("Drone odometry not yet received.")

            except Exception as e:
                self.get_logger().warn(f"Failed to publish landing pose: {e}")
                rclpy.shutdown()
                
        elif A> 6000:
            M = cv2.moments(c)
            if M['m00']==0: 
                # continue
                return None
            cx = int(M['m10']/M['m00']); cy = int(M['m01']/M['m00'])
            cv2.drawContours(frame,[c],-1,(0,0,255),2)
            cv2.circle(frame,(cx,cy),4,(0,0,255),-1)
            self.image = frame
            self.frame = frame.copy()
            self.annotated_image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, 'bgr8'))
            return {'center':(cx,cy),'area':A,'shape':'stop'}
        return None
        
    def control_loop(self):

        if self.state == 'ALIGN_HYDRANT':
            if self.latest is None:
                self.state = 'SEARCH'
                return

            cx, cy = self.latest['center']
            h, w = self.frame.shape[:2]
            ex = (cx - w / 2) / (w / 2)   # horizontal error
            ey = (h / 2 - cy) / (h / 2)   # vertical error

            cmd = Twist()
            aligned_x = abs(ex) < self.dead_x
            aligned_y = abs(ey) < self.dead_y

            # Always align in x
            if not aligned_x:
                cmd.angular.z = -0.4 * np.tanh(2.5 * ex)

            # Align in z only if object is close enough becuase the obstacles in the map do not allow the drone to go forward
            if self.area >= 60000 and not aligned_y:
                cmd.linear.z = 0.3 * np.tanh(2.5 * ey)

            # Transition to FORWARD if aligned
            if aligned_x and (self.area < 60000 or aligned_y):
                self.centered_count += 1
                if self.centered_count >= self.centered_frames:
                    self.get_logger().info("HYDRANT centered. Moving forward.")
                    self.state = 'FORWARD_HYDRANT'
                    self.forward_start = time.time()
                    self.cmd_pub.publish(Twist())  # stop movement
                else:
                    self.cmd_pub.publish(cmd)
            else:
                self.centered_count = 0
                self.cmd_pub.publish(cmd)
            return






        if self.state=='FORWARD_HYDRANT':
            if time.time()-self.forward_start < self.stop_forward_dur:
                cmd=Twist(); cmd.linear.x=0.05
                self.cmd_pub.publish(cmd)
                self.state='ALIGN_HYDRANT'
            else:
                self.cmd_pub.publish(Twist()); time.sleep(0.1)

            return

        # GATE SEARCH / ALIGN / FORWARD
        if self.state=='SEARCH':
            if self.latest:
                self.cmd_pub.publish(Twist()); time.sleep(0.1)
                self.state='ALIGN_HYDRANT'; self.centered_count=0
            else:
                cmd=Twist(); cmd.angular.z=0.1
                self.cmd_pub.publish(cmd)
            return
 


    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = DroneRacing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.land()
        rclpy.shutdown()

if __name__=='__main__':
    main()
