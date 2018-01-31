#!/usr/bin/env python
import rospy
import numpy as np
import math
from turtlebot3_auto_msgs.msg import  Twist2DStamped, LanePose, ImgSignals
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from time import sleep
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Float64

class tunnel_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED,
        self.TunnelState = "WAIT"
        self.StartOdom = None
        self.CurOdom = None
        self.FirstCycle = True
        self.gostart_distance = 0
        self.SearchStop = None
        self.obstacle = False
        # Publicaiton
        self.pub_cmd_vel_tunnel = rospy.Publisher('/cmd_vel_tunnel', Twist, queue_size=5)
        self.pub_con_state_change = rospy.Publisher ("/con_state_change", String, queue_size=1)
        self.pub_motor_control_start = rospy.Publisher("/motor_control_start", Bool, queue_size=1)

        # Subscriptions
        self.sub_laser_scan = rospy.Subscriber ("/scan", LaserScan, self.cbLaserScan, queue_size=1)
        self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
        self.sub_odom_reading = rospy.Subscriber ("/con_state", String, self.cbConState, queue_size=1)

        ##### Line Detect Side
        #import time
        #self.start_time = rospy.get_time()
        #self.sub_motor_start = rospy.Subscriber("/motor_control_start", Bool, self.cbMotorControlStart, queue_size=1)
        #self.MotorStartTime = rospy.get_time()

    '''
    def cbMotorControlStart(self, start_msg) :
        #starttt = start_msg.data
        self.MotorStart = True
    def dstewtwt
        cur_time = rospy.get_time() 
        print(  "time_gap :", cur_time - self.start_time )        
    '''
    def cbOdometry(self, odom_msg):
        self.CurOdom = [self.odom_reading.pose.pose.position.x, self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w]
        if self.FirstCycle == True :
            self.StartOdom = self.CurOdom
            self.FirstCycle == False
            self.pub_motor_control_start.publish(True)

        if self.TunnelState == "START" :
            self.vehicleStop()
            self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "ANG_SEARCH_START" :
            twist = Twist();twist.linear.x = 0;twist.angular.z = 0.3;
            self.pub_cmd_vel_tunnel.publish(twist)
            sleep(2)
            self.gostart_distance = (self.CurOdom[0]-self.StartOdom[0])*(self.CurOdom[0]-self.StartOdom[0]) +  (self.CurOdom[1]-self.StartOdom[1])*(self.CurOdom[1]-self.StartOdom[1])
            self.TunnelState = "LIDAR_CHECK"

        if self.TunnelState == "LIDAR_CHECK":
            self.vehicleStop()
            sleep(0.3)
            if self.obstacle  == False :
                self.vehicleStop()
                self.TunnelState = "GO"
            else :
                self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "GO" :
            twist = Twist(); twist.linear.x = 0.05; twist.angular.z = 0.0;
            self.pub_cmd_vel_tunnel.publish(twist)
            self.current_distance = (self.CurOdom[0]-self.StartOdom[0])*(self.CurOdom[0]-self.StartOdom[0]) +  (self.CurOdom[1]-self.StartOdom[1])*(self.CurOdom[1]-self.StartOdom[1])
            if self.current_distance  >  self.gostart_distance + 300 :
                self.TunnelState == "ANG_SEARCH_START"
                self.vehicleStop()
            if self.obstacle == True :
                self.TunnelState == "ANG_SEARCH_START"
                self.vehicleStop()

    def vehicleStop(self):
        twist = Twist(); twist.linear.x = 0; twist.angular.z = 0;
        self.pub_cmd_vel_tunnel.publish(twist)

    def cbConState(self, con_state_msg):
        if self.TunnelState == "WAIT" and con_state_msg.data == "TUNNEL":
            self.TunnelState = "START"

    def cbLaserScan(self, laser_scan_msg):
        self.laser_scan = laser_scan_msg.ranges
        self.obstacle = False
        for i in range(-10,10) :
            if i < 0 :
                i = 360 - i
            if self.laser_scan[i] != 0 and self.laser_scan[i] < 0.5:
               rospy.loginfo("_%d , %0.3f" % (i, self.laser_scan[i]))
               self.obstacle = True

if __name__ == "__main__":
    rospy.init_node("tunnel_controller",anonymous=False)
    tunnel_control_node = tunnel_controller()
    rospy.spin()
