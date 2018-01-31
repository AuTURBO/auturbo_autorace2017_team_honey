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
from sensor_msgs.msg import Imu
from scipy.spatial import distance

class tunnel_controller(object):
#	def __init__(self):
#		self.node_name = rospy.get_name()	

    def __init__(self):
        self.node_name = rospy.get_name()

        self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED,
        self.TunnelState = "WAIT"
        self.StartOdom = None
        self.CurOdom = None
        self.FirstCycle = True
        self.gostart_distance = 0
        self.SearchStop = None
        self.frontObstacle = False
        self.leftObstacle = False
        self.frontObstacleincline = 0
        self.leftObstacleincline = 0
        # Publicaiton
        self.pub_cmd_vel_tunnel = rospy.Publisher('/cmd_vel_tunnel', Twist, queue_size=5)

        # Subscriptions
        self.sub_laser_scan = None
        self.sub_odom_reading = None
        self.sub_imu_reading = None
        self.sub_signal_reading = rospy.Subscriber("/signals", String, self.cbSignal, queue_size=1)
        #rospy.Timer(rospy.Duration.from_sec(3.0), self.send_motor_start)
        #self.sub_laser_scan = rospy.Subscriber("/scan", LaserScan, self.cbLaserScan, queue_size=1)
        self.sub_laser_scan = None
    def cbSignal(self, signal_msg):
        if signal_msg.data == "TUNNEL":
            if self.sub_laser_scan == None :
                self.TunnelState = "START"
                self.sub_laser_scan = rospy.Subscriber("/scan", LaserScan, self.cbLaserScan, queue_size=1)
                self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
                self.sub_imu_reading = rospy.Subscriber("/imu", Imu, self.cbImuState, queue_size=1)

    def cbImuState(self,imu_msg):
        imu = [imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w]
        #print("cbImuState : ",imu)


    def cbOdometry(self, odom_msg):
        self.CurOdom = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.orientation.w]
        if self.FirstCycle == True :
            print("FirstCycle ...............")
            self.FirstCycle == False

        if self.TunnelState == "START" :
            print("START ...............")
            self.vehicleStop()
            self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "ANG_SEARCH_START" :
            print("ANG_SEARCH_START ...............")
            twist = Twist();twist.linear.x = 0;twist.angular.z = -0.2;
            self.pub_cmd_vel_tunnel.publish(twist)
            sleep(1.5)
            self.TunnelState = "LIDAR_CHECK"

        if self.TunnelState == "LIDAR_CHECK":
            print("LIDAR_CHECK ...............")
            self.vehicleStop()
            sleep(0.3)
            if self.frontObstacle  == False :
                self.vehicleStop()
                self.TunnelState = "GO"
            else :
                self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "GO" :

            twist = Twist();
            twist.linear.x = 0.05; twist.angular.z = 0.0;
            if self.leftObstacleincline > 0.005 :
                twist.angular.z = 0.1;
                print("GO ...............1", self.leftObstacleincline)
            elif self.leftObstacleincline < - 0.007 :
                twist.angular.z = -0.1;
                print("GO ...............2", self.leftObstacleincline)
            else :
                print("GO ...............3", self.leftObstacleincline)
            self.pub_cmd_vel_tunnel.publish(twist)
            #sleep(0.1)

            if self.frontObstacle == True : #or self.leftObstacle == True:
                self.TunnelState = "ANG_SEARCH_START"
                self.vehicleStop()
                print("GO ...............2", self.frontObstacle, self.leftObstacle )

    '''
    def cbOdometry(self, odom_msg):
        self.CurOdom = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.orientation.w]
        if self.FirstCycle == True :
            print("FirstCycle ...............")
            self.StartOdom = self.CurOdom
            self.StartOdom = [ 0.5710443258285522, -1.4218412637710571]
            self.FirstCycle = False

        P1 = (self.CurOdom[0], self.CurOdom[1])
        P2 = (self.StartOdom[0], self.StartOdom[1])
        dst = distance.euclidean(P1, P2)

        print(dst, self.CurOdom[0], self.CurOdom[1],self.StartOdom[0],self.StartOdom[1] )
        #return

        if self.TunnelState == "START" :
            print("START ...............")
            self.vehicleStop()
            self.TunnelState = "ANG_SEARCH_START"

        if self.TunnelState == "ANG_SEARCH_START" :
            print("ANG_SEARCH_START ...............")
            twist = Twist();twist.linear.x = 0;twist.angular.z = -0.2;
            self.pub_cmd_vel_tunnel.publish(twist)
            sleep(2)
            self.gostart_distance = dst
            self.TunnelState = "LIDAR_CHECK"

        if self.TunnelState == "LIDAR_CHECK":
            print("LIDAR_CHECK ...............")
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
            #sleep(0.1)
            self.current_distance = dst
            print("GO ...............", self.current_distance, self.gostart_distance, self.obstacle)

            if self.obstacle == True :
                self.TunnelState = "ANG_SEARCH_START"
                self.vehicleStop()
                print("GO ...............2", self.current_distance, self.gostart_distance, self.obstacle)
        '''

    def cbLaserScan(self, laser_scan_msg):
        #return
        self.laser_scan = laser_scan_msg.ranges
        self.frontObstacle = False
        front_count = 0
        left_count = 0

        for i in range(0,20) :
            if i < 10 :
               d = 350 + i
            else :
                d = i - 10
            if self.laser_scan[d] != 0 and self.laser_scan[d] < 0.15:
               #rospy.loginfo("_%d , %0.3f" % (d, self.laser_scan[d]))
               self.frontObstacle = True
            elif self.laser_scan[d] == 0 :
                front_count = front_count + 1

        self.frontObstacleincline = self.laser_scan[350] - self.laser_scan[9]

        self.leftObstacle = False

        for i in range(90-5,90+5) :
            if self.laser_scan[i] != 0 and self.laser_scan[i] < 0.15:
               rospy.loginfo("55_%d , %0.3f" % (i, self.laser_scan[i]))
               self.leftObstacle = True
            elif self.laser_scan[i] == 0 :
                left_count = left_count + 1

        self.leftObstacleincline = self.laser_scan[80] - self.laser_scan[99]

        if front_count > 15 :
            #self.frontObstacle = True
            print("front obstacle near")

        if left_count > 15:
            #self.leftObstacle = True
            print("left obstacle near")

        print("obstacle Front " , self.frontObstacle,",Left ",self.leftObstacle, self.leftObstacleincline )

    def vehicleStop(self):
        twist = Twist(); twist.linear.x = 0; twist.angular.z = 0;
        self.pub_cmd_vel_tunnel.publish(twist)

    def onShutdown(self):
        self.loginfo("tunnel motor Shutdown.")
        if self.sub_laser_scan != None :
            self.sub_laser_scan.unregister()
        if self.sub_odom_reading != None:
            self.sub_odom_reading.unregister()
        if self.sub_imu_reading != None:
            self.sub_imu_reading.unregister()
        if self.sub_signal_reading != None:
            self.sub_signal_reading.unregister()

        #self.Timer.shutdown()

if __name__ == "__main__":
	rospy.init_node("tunnel_controller",anonymous=False)
	tunnel_control_node = tunnel_controller()
	rospy.spin()
