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

class motor_controller(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.cmd_vel_line_reading = None
		self.cmd_vel_parking_reading = None
		self.cmd_vel_tunnel_reading = None
		self.signal_reading = None
		self.odom_reading = None

		self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED,
		#self.con_state = "TUNNEL"  # LINE, BAR, PARKING, TUNNEL, LED,
		#self.prev_con_state = "LINE"
		self.motor_stop_state = False
		self.led_state = None
		self.bar_state = None
		self.pub_counter = 0
		self.tunneltimetrigger = False

		self.frontObstacle = False
		self.leftObstacle = False
		self.frontObstacleincline = 0
		self.leftObstacleincline = 0

		self.cmd_vel_line = Twist()
		self.cmd_vel_tunnel = Twist()
		self.cmd_vel_parking = Twist()
		self.cmd_vel_bar = Twist()

		# Setup parameters
		#self.setGains()

		# Publicaiton
		self.pub_car_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.pub_con_state = rospy.Publisher('/con_state', String,queue_size=1)

		# Subscriptions
		# self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
		#self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
		self.sub_cmd_vel_line_reading = rospy.Subscriber("/cmd_vel_line", Twist, self.cbCmdVelLine, queue_size=1)
		self.sub_cmd_vel_parking_reading = rospy.Subscriber("/cmd_vel_parking", Twist, self.cbCmdVelParking, queue_size=1)
		self.sub_cmd_vel_tunnel_reading = rospy.Subscriber("/cmd_vel_tunnel", Twist, self.cbCmdVelTunnel, queue_size=1)
		self.sub_signal_reading = rospy.Subscriber("/signals", String, self.cbSignal, queue_size=1)
		#self.sub_state_change = rospy.Subscriber("/con_state_change", String, self.cbConStateChange, queue_size=1)
		#self.sub_line_state_reading = rospy.Subscriber("/line_state", String, self.cbLineState, queue_size=1)
		self.sub_laser_scan = rospy.Subscriber("/scan", LaserScan, self.cbLaserScan, queue_size=1)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
		rospy.Timer(rospy.Duration.from_sec(0.1), self.drive_control)
		self.redtime = rospy.get_time()

	def drive_control(self, _event):
		if self.con_state == "LINE":
			twist = Twist()
			twist.linear.x = self.cmd_vel_line.linear.x
			twist.angular.z = self.cmd_vel_line.angular.z
			self.pub_car_cmd.publish(twist)
			print("LINE...............", twist.linear.x, twist.angular.z)
		elif self.con_state == "TUNNEL":
			twist = Twist()
			twist.linear.x = self.cmd_vel_tunnel.linear.x
			twist.angular.z = self.cmd_vel_tunnel.angular.z
			self.pub_car_cmd.publish(twist)
		elif self.con_state == "PARKING":
			twist = Twist()
			twist.linear.x = self.cmd_vel_parking.linear.x
			twist.angular.z = self.cmd_vel_parking.angular.z
			self.pub_car_cmd.publish(twist)
		elif self.con_state == "BAR":
			twist = Twist()
			twist.linear.x = self.cmd_vel_bar.linear.x
			twist.angular.z = self.cmd_vel_bar.angular.z
			self.pub_car_cmd.publish(twist)
		elif self.con_state == "RED":
			nowtime = rospy.get_time()
			if ( nowtime - self.redtime ) > 1 :
				#print("RED OUT ...............")
				self.con_state = "LINE"

	def cbLaserScan(self, laser_scan_msg):
		# return
		self.laser_scan = laser_scan_msg.ranges
		self.frontObstacle = False
		front_count = 0
		left_count = 0

		for i in range(0, 20):
			if i < 10:
				d = 350 + i
			else:
				d = i - 10
			if self.laser_scan[d] != 0 and self.laser_scan[d] < 0.1:
				rospy.loginfo("_%d , %0.3f" % (d, self.laser_scan[d]))
				self.frontObstacle = True
			elif self.laser_scan[d] == 0:
				front_count = front_count + 1

		self.frontObstacleincline = self.laser_scan[350] - self.laser_scan[9]

		self.leftObstacle = False

		for i in range(90 - 5, 90 + 5):
			if self.laser_scan[i] != 0 and self.laser_scan[i] < 0.15:
				#rospy.loginfo("55_%d , %0.3f" % (i, self.laser_scan[i]))
				self.leftObstacle = True
			elif self.laser_scan[i] == 0:
				left_count = left_count + 1

		self.leftObstacleincline = self.laser_scan[80] - self.laser_scan[99]

		#if front_count > 15:
			# self.frontObstacle = True
			#print("front obstacle near")

		#if left_count > 15:
			# self.leftObstacle = True
			#print("left obstacle near")

		#print("obstacle Front ", self.frontObstacle, ",Left ", self.leftObstacle, self.leftObstacleincline)

	def vehicleStop(self):
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
		self.pub_car_cmd.publish(twist)

	def custom_shutdown(self):
		# Stop listening
		self.sub_signal_reading.unregister()
		self.sub_cmd_vel_line_reading.unregister()
		self.sub_cmd_vel_tunnel_reading.unregister()

		self.vehicleStop()
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbSignal(self,signal_msg):
		if signal_msg.data == "BAR" :
			self.con_state = "BAR"
		elif signal_msg.data == "RED" :
			self.con_state = "RED"
			self.vehicleStop()
			self.redtime = rospy.get_time()
			print("RED contorl start ....$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
		elif signal_msg.data == "PARKING" :
			self.con_state = "PARKING"
		elif signal_msg.data == "TUNNEL" :
			sleep(10)
			print("tunnel contorl start ....$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
			self.con_state = "TUNNEL"
		elif signal_msg.data == "LED" :
			self.con_state = "LED"
			#self.led_state = self.signal_reading.LED #RED,YELLOW,GREEN
		#self.pub_con_state.publish(self.con_state)
		#sleep(0.1)

	def cbCmdVelLine(self, cmd_vel_line_msg):
		self.cmd_vel_line.linear.x = cmd_vel_line_msg.linear.x
		self.cmd_vel_line.angular.z = cmd_vel_line_msg.angular.z

	def cbCmdVelParking(self, cmd_vel_parking_msg):
		self.cmd_vel_parking.linear.x = cmd_vel_parking_msg.linear.x
		self.cmd_vel_parking.angular.z = cmd_vel_parking_msg.angular.z

	def cbCmdVelTunnel(self, cmd_vel_tunnel_msg):
		self.cmd_vel_tunnel.linear.x = cmd_vel_tunnel_msg.linear.x
		self.cmd_vel_tunnel.angular.z = cmd_vel_tunnel_msg.angular.z
		'''
		if self.con_state == "LED":
			self.controlByLed(self.led_state)
		elif self.con_state == "BAR":
			self.controlByLed(self.bar_state)
		'''
	'''
	def controlByLed(self,led_state):
		if led_state == "LED" or led_state == "YELLOW" :
			self.vehicleStop()
		elif led_state == "GREEN" :
			self.con_state = "LINE"
		self.pub_con_state.publish(self.con_state)

	def controlByBar(self,bar_state):
		if bar_state == "HOR" :
			self.vehicleStop()
		elif bar_state == "VER" :
			self.con_state = "LINE"
		self.pub_con_state.publish(self.con_state)
	'''
if __name__ == "__main__":
	rospy.init_node("motor_controller",anonymous=False)
	motor_control_node = motor_controller()
rospy.spin()
