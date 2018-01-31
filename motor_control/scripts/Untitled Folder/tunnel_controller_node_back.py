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
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import time

class tunnel_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.cmd_vel_line_reading = None
        self.cmd_vel_parking_reading = None
        self.cmd_vel_tunnel_reading = None
        self.signal_reading = None
        self.odom_reading = None

        self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED, 
        self.motor_stop_state = None
        self.led_state = None
        self.bar_state = None
        self.pub_counter = 0

        # Setup parameters
        #self.setGains()

        # Publicaiton
        self.pub_cmd_vel_tunnel = rospy.Publisher('/cmd_vel_tunnel', Twist, queue_size=5)
        #self.pub_con_state = rospy.Publisher('/con_state', String,queue_size=1)

        # Subscriptions
        # self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        #self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
        #self.sub_cmd_vel_line_reading = rospy.Subscriber("/cmd_vel_line", Twist, self.cbCmdVelLine, queue_size=1)

        self.sub_cmd_vel_parking_reading = rospy.Subscriber("/cmd_vel_parking", Twist, self.cbCmdVelParking, queue_size=1)
        self.sub_cmd_vel_tunnel_reading = rospy.Subscriber("/cmd_vel_tunnel", Twist, self.cbCmdVelTunnel, queue_size=1)
        self.sub_signal_reading = rospy.Subscriber("/signals", ImgSignals, self.cbSignal, queue_size=1)
        self.sub_state_change = rospy.Subscriber("/con_state_change", String, self.cbConStateChange, queue_size=1)
        self.sub_motor_stop = rospy.Subscriber("/motor_stop", Bool, self.cbMotorStop, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        #self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))


def vehicleStop(self):
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    self.publishCmd(twist)

def custom_shutdown(self):
    # Stop listening
    self.sub_odom_reading.unregister()
    self.sub_signal_reading.unregister()
    self.sub_cmd_vel_line_reading.unregister()
    self.sub_cmd_vel_parking_reading.unregister()
    self.sub_cmd_vel_tunnel_reading.unregister()
    # Send stop command
    self.vehicleStop()
    rospy.loginfo("[%s] Shutdown" %self.node_name)

def publishCmd(self,twist):
    if self.motor_stop_state :
        twist.linear.x = 0 ; twist.linear.y = 0 ; twist.linear.z = 0 ;
        twist.angular.x = 0 ; twist.angular.y = 0 ; twist.angular.z = 0 ;
    self.pub_car_cmd.publish(twist)


def cbCmdVelLine(self,cmd_vel_line_msg):
    self.cmd_vel_line_reading = cmd_vel_line_msg
    twist = Twist()
    twist.linear.x = self.cmd_vel_line_reading.linear.x
    twist.angular.z = self.cmd_vel_line_reading.angular.z
    if self.con_state == "LINE" :
        self.publishCmd(twist)

def cbCmdVelParking(self,cmd_vel_parking_msg):
    self.cmd_vel_parking_reading = cmd_vel_parking_msg
    twist = Twist()
    twist.linear.x = self.cmd_vel_line_reading.linear.x
    twist.angular.z = self.cmd_vel_line_reading.angular.z
    if self.con_state == "PARKING" :
        self.publishCmd(twist)

def cbCmdVelTunnel(self,cmd_vel_tunnel_msg):
    self.cmd_vel_tunnel_reading = cmd_vel_tunnel_msg
    twist = Twist()
    twist.linear.x = self.cmd_vel_line_reading.linear.x
    twist.angular.z = self.cmd_vel_line_reading.angular.z
    if self.con_state == "TUNNEL" :
        self.publishCmd(twist)

def cbOdometry(self,odom_msg):
    self.odom_reading = odom_msg
    #rospy.loginfo('x: %f , y:%f , w:%f ',self.odom_reading.pose.pose.position.x ,  self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w )

def cbSignal(self,signal_msg):
    self.signal_reading = signal_msg
    rospy.loginfo('Signal: %s , LED : %s ',self.signal_reading.SIGNAL , self.signal_reading.LED )
    if self.signal_reading.SIGNAL == "BAR" :
        self.con_state = "BAR"
        self.bar_state = self.signal_reading.BAR #VER, HOR
    elif self.signal_reading.SIGNAL == "PARKING" :
        self.con_state = "PARKING"
    elif self.signal_reading.SIGNAL == "TUNNEL" :
        self.con_state = "TUNNEL"
    elif self.signal_reading.SIGNAL == "LED" :
        self.con_state = "LED"
        self.led_state = self.signal_reading.LED #RED,YELLOW,GREEN
    self.pub_con_state.publish(self.con_state)
    #sleep(0.1)

    if self.con_state == "LED":
        controlByLed(self.led_state)
    elif self.con_state == "BAR":
        controlByLed(self.bar_state)

def cbConStateChange(self,con_state_change):
    if con_state_change == "TUNNEL_OUT" :
        self.con_state = "LINE"
    elif self.signal_reading.SIGNAL == "PARKING_OUT" :
        self.con_state = "LINE"
    self.pub_con_state.publish(self.con_state)

def cbMotorStop(self,MotorStop):
    if MotorStop :
        self.motor_stop_state = True
    else :
        self.motor_stop_state = False
    sleep(0.5)
    self.vehicleStop()

def controlByLed(self,led_state):
    if led_state == "LED" or led_state == "YELLOW" :
        vehicleStop()
    elif led_state == "GREEN" :
        self.con_state = "LINE"
    self.pub_con_state.publish(self.con_state)

def controlByBar(self,bar_state):
    if bar_state == "HOR" :
        vehicleStop()
    elif led_state == "VER" :
        self.con_state = "LINE"
    self.pub_con_state.publish(self.con_state)

if __name__ == "__main__":
rospy.init_node("tunnel_controller",anonymous=False)
tunnel_control_node = tunnel_controller()
rospy.spin()
