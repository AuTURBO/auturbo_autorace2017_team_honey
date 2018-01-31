#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from turtlebot3_auto_msgs.msg import SegmentList, Segment
from scipy.stats import multivariate_normal, entropy
from scipy.ndimage.filters import gaussian_filter
from math import floor, atan2, pi, cos, sin, sqrt
import time
from geometry_msgs.msg import Twist
from time import sleep

class LaneFilterNode(object):
	"""

Lane Filter Node

Author: Liam Paull

Inputs: SegmentList from line detector

Outputs: LanePose - the d (lateral displacement) and phi (relative angle)
of the car in the lane

For more info on algorithm and parameters please refer to the google doc:
 https://drive.google.com/open?id=0B49dGT7ubfmSX1k5ZVN1dEU4M2M

    """
	def __init__(self):
		self.node_name = "Lane Filter"
		self.active = True
		self.white_x = 0
		self.white_y = 0
		self.white_length = 0
		self.white_degree = 0
		self.white_time = rospy.get_time()

		self.yellow_x = 0
		self.yellow_y = 0
		self.yellow_length = 0
		self.yellow_degree = 0
		self.yellow_time = rospy.get_time()

		# Subscribers
		self.sub = rospy.Subscriber("segment_list", SegmentList, self.processSegments, queue_size=1)

		# Publishers
		self.pub_car_cmd = rospy.Publisher('/cmd_vel_line', Twist, queue_size=5)

		rospy.Timer(rospy.Duration.from_sec(1.0), self.line_check)


	def line_check(self,_event) :
		cur_time = rospy.get_time()
		if 0 != self.yellow_time and  0 != self.white_time :
			if cur_time - self.yellow_time >2 and cur_time - self.white_time >2 :
				print("there is no line !!!!!!!!!!!")
				twist_n = Twist(); twist_n.linear.x = 0.02;twist_n.angular.z = 0.1;
				self.pub_car_cmd.publish(twist_n)

	def processSegments(self,segment_list_msg):
		if not self.active:
			return
		t_start = rospy.get_time()

		for segment in segment_list_msg.segments:
			if segment.color != segment.WHITE and segment.color != segment.YELLOW:
				continue
			if segment.points[0].x < 0 or segment.points[1].x < 0:
				continue
			self.generateVote(segment)

	def generateVote(self,segment):

		angular = 0
		velocity = 0

		# P1 : Yellow0 Start Point , P2 : Yellow1 End Point
	    # arrange ,  x value of P1 is lower than x value of P2

		if segment.color == segment.YELLOW:
			self.yellow_time = rospy.get_time()
			if segment.pixels_normalized[0].x <= segment.pixels_normalized[1].x:
				p1 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])
				p2 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
			else:
				p1 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
				p2 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])

			# check infinite value
			delta_p = p2-p1
			if delta_p[0] == 0 :
				rospy.loginfo( "delta_p[0] isinfinite happen1" )
				##this is need at Sharp curve like Bar yellow curve Line
				twist_tmp = Twist()
				twist_tmp.linear.x = 0.05 ;  twist_tmp.angular.z = 0;
				self.pub_car_cmd.publish(twist_tmp)
				sleep(0.5);
				self.onVehicleStop()
				return

			# caluration inclination
			inclination = abs(delta_p[1] / delta_p[0])

			# caluration x,y intercep
			extend_y = inclination*p1[0] + p1[1]
			if inclination == 0 :
				rospy.loginfo( "inclination isinfinite happen2" )
				##this is need at Sharp curve like Bar yellow curve Line
				twist_tmp = Twist()
				twist_tmp.linear.x = 0.05 ;  twist_tmp.angular.z = 0;
				self.pub_car_cmd.publish(twist_tmp)
				sleep(0.5);
				self.onVehicleStop()
				return
			extend_x = extend_y/inclination
			# caluration yellow line distance by x,y intercep
			extend_length = extend_x*extend_x + extend_y*extend_y
			# caluration yellow degree
			extend_degree =  np.arctan([extend_y,extend_x])

			if extend_x<0 or extend_y<0 :
				self.onVehicleStop()
				rospy.loginfo( "Unexpected situation. So Robot Stop1 " )

			# TUNNING POINT
			beta = 0.85 # alpha blending
			angular = 0.0 # TB3 rotation value, +value is left rotation , -value is rigt rotation ,
			velocity = 0.07 # TB3 Go,Back Speed, +value is Go , -value is Back
			#velocity = 0.08
			#velocity = 0.07

			#apply past values
			self.yellow_x = beta*extend_x + (1 - beta)*self.yellow_x
			self.yellow_y = beta*extend_y + (1 - beta)*self.yellow_y
			self.yellow_length = beta*extend_length + (1 - beta)*self.yellow_length
			self.yellow_degree = beta*extend_degree + (1 - beta)*self.yellow_degree

			# Now there is yellow line,
			# Check the distance from the yellow line, and determine the degree of rotation of the circle.
			# if inclination yellow_length is long -> TB3 , The robot is near to the yellow line.-> decreas angular
			# if inclination yellow_length is short -> TB3 , The robot is far to the yellow line.-> increas angular
			if self.yellow_length <= 1500 :
				angular= 0.25
			elif 1500 < self.yellow_length and self.yellow_length <= 2500 :
				angular =  0.15
			elif 2500 < self.yellow_length and self.yellow_length <= 3500 :
				angular =  0.10
			elif 3500 < self.yellow_length and self.yellow_length <= 6500 :
				angular =  0
			elif 6500 < self.yellow_length and self.yellow_length <= 7500 :
				angular = -0.10
			elif 7500 < self.yellow_length and self.yellow_length <= 8500 :
				angular = -0.15
			elif 8500 < self.yellow_length and self.yellow_length <= 15000 :
				angular = -0.22
			elif 15000 < self.yellow_length :
				angular = -0.25  #0.3
				velocity = 0.03 #0.03

			twist_y = Twist()
			twist_y.linear.x = velocity; twist_y.angular.z = angular;
			self.pub_car_cmd.publish(twist_y)
			rospy.loginfo( ' yellow_length %.0f , arg %.1f ',self.yellow_length , angular )

		elif segment.color == segment.WHITE:
			self.white_time = rospy.get_time()
			if (self.white_time - self.yellow_time) > 0.3:
				# Now there is only white line , no yellow line
				#rospy.loginfo( 'only white.time_di %.2f',self.white_length , angular, ( self.white_time - self.yellow_time ) )
				rospy.loginfo( 'only white line detect_yellow line is detected before %.2f sec ', self.white_time - self.yellow_time)
				twist_w = Twist()
				twist_w.linear.x = 0.03; twist_w.angular.z = 0.2;
				self.pub_car_cmd.publish(twist_w)
				##this is need at Sharp curve like Bar yellow curve Line
				sleep(0.2);
				twist_w.linear.x = 0.03; twist_w.angular.z = 0;
				self.pub_car_cmd.publish(twist_w)
				sleep(0.1);
				self.onVehicleStop()
				#self.onVehicleStop()

			else :
				rospy.loginfo(' suspend only white line ,  yellow line is detected before %.2f  ', self.white_time - self.yellow_time)
				rospy.loginfo('only white line detect_yellow line is detected before %.2f sec ',
							  self.white_time - self.yellow_time)
				'''
				twist_w = Twist()
				twist_w.linear.x = 0.03;
				twist_w.angular.z = 0.2;
				self.pub_car_cmd.publish(twist_w)
				##this is need at Sharp curve like Bar yellow curve Line
				sleep(0.2);
				twist_w.linear.x = 0.032;
				twist_w.angular.z = 0;
				self.pub_car_cmd.publish(twist_w)
				sleep(0.1);
				self.onVehicleStop()
				'''
		#send motor control node

		#rospy.loginfo('publish velocity %.2f , argural %.1f',velocity, angular )
		return

	def onVehicleStop(self):
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		self.pub_car_cmd.publish(twist)

	def onShutdown(self):
		self.onVehicleStop()
		self.sub.unregister()
		rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__':
	rospy.init_node('lane_filter',anonymous=False)
	lane_filter_node = LaneFilterNode()
	rospy.on_shutdown(lane_filter_node.onShutdown)
	rospy.spin()
