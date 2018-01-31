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

class parking_controller(object):
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
		self.ParkingState = "WAIT"
		self.yellow_degree = 0
		self.yellow_diappear_count = 0
		#self.ParkingTargetP = []
		self.laser_scan = None
		self.yellow_degreelist = []
		self.obstacle = False
		self.zero_count = 0
		self.last_dot_line = True
		self.SecondTry = False
		self.dot_disapear = False
		self.dot_time = False

		# Setup parameters
		#self.setGains()

		# Publicaiton
		self.pub_cmd_vel_parking = rospy.Publisher('/cmd_vel_parking', Twist, queue_size=5)
		self.pub_con_state_change = rospy.Publisher("/con_state_change", String, queue_size=1)

		# Subscriptions
		self.sub_con_state_reading = rospy.Subscriber("/con_state", String, self.cbConState, queue_size=1)
		self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
		self.sub_line_state_reading = rospy.Subscriber("/dot_state", String, self.cbLineState, queue_size=1)
		self.sub_yellow_degree = rospy.Subscriber("/yellow_degree", Int32, self.cbYellowSDegree, queue_size=1)
		self.sub_laser_scan = rospy.Subscriber("/scan", LaserScan, self.cbLaserScan, queue_size=1)

	def cbLaserScan(self,laser_scan_msg):
		self.laser_scan = laser_scan_msg.ranges
		object_count = 0;
		#return
		test = "start//////////////////////////////////////////////////////////////"
		#for i in range(len(laser_scan_msg.ranges)):
		for i in range(180-10,180+10):
			#test= test+ "," + str(i)+":"+ str( round(self.laser_scan[i],2) )
			#test= test+ "," + str( round(self.laser_scan[i]*10) )
			if self.laser_scan[i] != 0  and  self.laser_scan[i] < 0.38 :
				#rospy.loginfo("_%d , %0.3f" %(i, self.laser_scan[i]))
				object_count = object_count + 1
		if object_count == 0 :
			self.obstacle = False
		else :
			self.obstacle = True
			#print("there is obect !!!!!!!!!!!  ")

	def cbYellowSDegree(self,yellow_degree_msg):
		yellow_deg = yellow_degree_msg.data
		if 99887 != yellow_deg :
			self.yellow_degreelist.append(yellow_deg)
			if len(self.yellow_degreelist) > 2 :
				self.yellow_degreelist.pop(0)

		self.yellow_degree = np.mean(self.yellow_degreelist)
	#print( self.yellow_degreelist,  self.yellow_degree )

	def vehicleStop(self):
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
		self.pub_cmd_vel_parking.publish(twist)

	def custom_shutdown(self):
		# Stop listening
		self.sub_odom_reading.unregister()
		self.sub_con_state_reading.unregister()
		# Send stop command
		self.vehicleStop()
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbConState(self, con_state_msg ):
		if self.ParkingState == "WAIT" and con_state_msg.data == "PARKING" and self.SecondTry == False :
			#self.ParkingState = "START"
			self.ParkingState = "START_IN_ROTATION"

	def cbLineState(self, line_state_msg ):
		test = line_state_msg
		print("cbLineState........tttttt",  line_state_msg.data )
		if line_state_msg.data == "DOT_LINE" :
			self.last_dot_line = True
			self.dot_disapear = False
			self.dot_time = rospy.get_time()

	def cbOdometry(self,odom_msg):
		self.odom_reading = odom_msg
		OdomP1 = [ self.odom_reading.pose.pose.position.x , self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w ]
		cur_dot_time = rospy.get_time()

		if cur_dot_time - self.dot_time > 2 :
			self.dot_disapear = True

		if self.dot_disapear == True and self.SecondTry == True and self.ParkingState == "STOP":
			self.pub_con_state_change.publish("PARKING2")
			self.vehicleStop()
			self.ParkingState = "START_IN_ROTATION"
			print( "dot distance", cur_dot_time - self.dot_time )

		#rospy.loginfo('x: %f , y:%f , w:%f _stat %s',self.odom_reading.pose.pose.position.x ,  self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w , self.ParkingState)

		if self.ParkingState == "START" :
			print( "START ")
			#return
			self.vehicleStop()
			sleep(1)
			self.ParkingStartP = OdomP1
			self.ParkingState = "START_IN_ROTATION"

		if self.ParkingState == "START_IN_ROTATION" :

			#return
			twist = Twist()
			zero_count = 0

			if self.yellow_degree <= -30 :
				twist.linear.x = 0; twist.angular.z = 0.2;
				self.pub_cmd_vel_parking.publish(twist)
				self.zero_count = 0
				print("START_IN_ROTATION1",self.yellow_degree)
			elif -30< self.yellow_degree  and self.yellow_degree < -0.5 :
				twist.linear.x = 0; twist.angular.z = 0.05;
				self.pub_cmd_vel_parking.publish(twist)
				print("START_IN_ROTATION2",self.yellow_degree)
			elif self.yellow_degree > 0.5 :
				twist.linear.x = 0; twist.angular.z = -0.05;
				self.pub_cmd_vel_parking.publish(twist)
				print("START_IN_ROTATION3",self.yellow_degree)
			elif -0.5 <= self.yellow_degree and self.yellow_degree <= 0.5 :
				self.vehicleStop()
				self.zero_count = self.zero_count + 1
				print("START_IN_ROTATION5",self.yellow_degree)
				if self.zero_count > 30 :
					self.vehicleStop()
					sleep(0.1)
					if self.obstacle :
						#self.SecondTry = True
						self.ParkingState = "START_OUT_ROTATION"
						print("test is some object")
					else :
						self.ParkingState = "START_IN_GO"
						self.last_dot_line = False

		if self.ParkingState == "START_IN_GO" :
			print("START_IN_GO ", self.last_dot_line )
			twist = Twist()
			if self.last_dot_line :
				sleep(1)
				self.vehicleStop()
				sleep(1)
				self.ParkingState = "START_OUT_GO"
			else :
				twist.linear.x = -0.02
				self.pub_cmd_vel_parking.publish(twist)

		if self.ParkingState == "START_OUT_GO" :
			#print("START_IN_GO ", (cur_time - self.last_dot_line_time))
			twist = Twist()
			twist.linear.x = 0.05
			self.pub_cmd_vel_parking.publish(twist)
			sleep(4.5)
			self.vehicleStop()
			sleep(1)
			self.ParkingState = "START_OUT_ROTATION"

		if self.ParkingState == "START_OUT_ROTATION" :
			print("START_OUT_ROTATION ")
			twist = Twist()
			twist.linear.x =  0; twist.angular.z = -0.3;
			self.pub_cmd_vel_parking.publish(twist)
			sleep(5.2)
			self.vehicleStop()
			sleep(2)
			self.ParkingState = "STOP"

		if self.ParkingState == "STOP" :
			#print("STOP")
			self.pub_con_state_change.publish("PARKING_OUT")
			sleep(1)

if __name__ == "__main__":
	rospy.init_node("parking_controller",anonymous=False)
	parking_control_node = parking_controller()
	rospy.spin()
