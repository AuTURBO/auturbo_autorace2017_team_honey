#!/usr/bin/env python
#from line_detector.AntiInstagram import *
from cv_bridge import CvBridge, CvBridgeError
from turtlebot3_auto_msgs.msg import ( BoolStamped, Segment,
                                       SegmentList, Vector2D)
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils.jpg import image_cv_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from line_detector.line_detector_plot import *
from line_detector.timekeeper import TimeKeeper
import cv2
import numpy as np
import rospy
import threading
import time
import yaml
from std_msgs.msg import Bool
from std_msgs.msg import String
import time

class LineDetectorNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"

        # Thread lock
        self.thread_lock = threading.Lock()

        # Constructor of line detector
        self.bridge = CvBridge()
        self.stats = Stats()
        self.detector = None
        self.detector2 = None
        self.updateParams(None)
        self.dark_count = 0

        # Only be verbose every 10 cycles
        self.intermittent_interval = 1
        self.intermittent_counter = 1

        # Publishers
        self.pub_lines = rospy.Publisher("segment_list", SegmentList, queue_size=1)
        self.pub_image_with_line = rospy.Publisher("image_with_lines", Image, queue_size=1)
        self.pub_image_with_line_o = rospy.Publisher("image_with_lines_o", Image, queue_size=1)
        #self.pub_image_with_line_o2 = rospy.Publisher("image_with_lines_o2", Image, queue_size=1)
        #self.pub_line_state = rospy.Publisher('/line_state', String, queue_size=1)
        self.pub_signal = rospy.Publisher('/signals', String, queue_size=1)
        #self.pub_image_origin = rospy.Publisher("image_with_origin", Image, queue_size=1)
        #self.pub_image_cal = rospy.Publisher("image_with_cal", Image, queue_size=1)

        # Subscribers
        self.sub_image = rospy.Subscriber("/camera1/image_raw/compressed", CompressedImage, self.cbImage, queue_size=1)
        #self.sub_image = rospy.Subscriber("/image_raw/compressed", CompressedImage, self.cbImage, queue_size=1)
        self.sub_motor_start = rospy.Subscriber("/motor_control_start", Bool, self.cbMotorControlStart, queue_size=1)

        rospy.loginfo("[%s] Initialized " %(self.node_name))
        #rospy.Line_Timer(rospy.Duration.from_sec(3.0), self.updateLineCheck)
        #rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)
        self.updateParams("test")

        with open("/home/rt/catkin_ws/src/turtlebot3_auto/line_detect/src/callibra_ros.yaml") as fr:
            self.c = yaml.load(fr)

    def updateParams(self,s):

        self.image_size = rospy.get_param('~img_size')
        self.top_cutoff = rospy.get_param('~top_cutoff')
        print("img_size", self.image_size[0], self.image_size[1],"cut_off",self.top_cutoff )
        #self.loginfo('top_cutoff = %d' % self.top_cutoff)

        if self.detector is None:
            c = rospy.get_param('~detector')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new detector config: %s' % str(c))
            self.detector = instantiate(c[0], c[1])

        if self.detector2 is None:
            c2 = rospy.get_param('~detector2')
            assert isinstance(c2, list) and len(c2) == 2, c2

            self.loginfo('new detector2 config: %s' % str(c2))
            self.detector2 = instantiate(c2[0], c2[1])

    def cbMotorControlStart(self, start_msg) :
        self.MotorStart = True

    def updateLineCheck(self, _event):
        return
        if self.Linedisable == True and self.MotorStart == True:
            #self.pub_line_state.publish("TUNNEL")
            print("here is tunnel.................................")
        self.Linedisable = True

    def cbImage(self, image_msg):
        self.stats.received()

        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def intermittent_log_now(self):
        return self.intermittent_counter % self.intermittent_interval == 0

    def intermittent_log(self, s):
        if not self.intermittent_log_now():
            return
        self.loginfo('%3d:%s' % (self.intermittent_counter, s))

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            self.stats.skipped()
            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()

    def processImage_(self, image_msg):

        self.stats.processed()

        if self.intermittent_log_now():
            #self.intermittent_log(self.stats.info())
            self.stats.reset()

        self.intermittent_counter += 1

        # Decode from compressed image with OpenCV
        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        #dark count and tunnel check
        hist = cv2.calcHist([image_cv], [0], None, [5], [0, 5])
        if (hist[0] + hist[1]) > 150000 :
            self.dark_count = self.dark_count + 1
        else :
            self.dark_count = 0

        if 3 < self.dark_count < 9 :
            self.pub_signal.publish("TUNNEL")



        #calibration proc && image publish
        K_undistort = np.array(self.c['camera_matrix'])
        img_und = cv2.undistort(image_cv, np.array(self.c['camera_matrix']), np.array(self.c['dist_coefs']),
                                newCameraMatrix=K_undistort)
        img_und_p = self.bridge.cv2_to_imgmsg(img_und, "bgr8")
        #self.pub_image_cal.publish(img_und_p)

        # Resize and crop image and rotation
        hei_original, wid_original = img_und.shape[0:2]

        if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            img_und = cv2.resize(img_und, (self.image_size[1], self.image_size[0]),
                                 interpolation=cv2.INTER_NEAREST)
            img_origin = cv2.resize(image_cv, (self.image_size[1], self.image_size[0]),
                                 interpolation=cv2.INTER_NEAREST)
        rotation_center = (self.image_size[1]/2, self.image_size[0]/2)
        rotation_mat = cv2.getRotationMatrix2D(rotation_center, 180, 1.0)
        img_und = cv2.warpAffine(img_und, rotation_mat, (self.image_size[1], self.image_size[0]))
        img_und = img_und[self.top_cutoff:,:,:]
        img_origin = cv2.warpAffine(img_origin, rotation_mat, (self.image_size[1], self.image_size[0]))
        img_origin2 = img_origin[:, :, :]
        img_origin = img_origin[:100,150:,:]
        image_cv_corr = cv2.convertScaleAbs(img_und)
        image_cv_corr_origin = cv2.convertScaleAbs(img_origin)
        image_cv_corr_origin2 = cv2.convertScaleAbs(img_origin2)

        ###############################################
        # Set the image to be detected
        self.detector.setImage(image_cv_corr)
        hei2_original, wid2_original = image_cv_corr.shape[0:2]

        # Detect lines and normals
        white = self.detector.detectLines('white')
        yellow = self.detector.detectLines('yellow')
        red = self.detector.detectLines('red')
        ################################################
        def detectLines2(self, color, hough_threshold, hough_min_line_length, hough_max_line_gap):
            bw, edge_color = self._colorFilter(color)

        #################################################
        self.detector2.setImage(image_cv_corr_origin)
        hei2_original_o, wid2_original_o = image_cv_corr_origin.shape[0:2]

        # Detect lines and normals
        #white_o = self.detector2.detectLines('white')
        #yellow_o = self.detector2.detectLines('yellow')
        #hough_threshold = 20
        #hough_min_line_length = 1
        #hough_max_line_gap = 1
        red_o = self.detector2.detectLines('red')

        #################################################3

        #################################################
        self.detector2.setImage(image_cv_corr_origin2)
        hei2_original_o2, wid2_original_o2 = image_cv_corr_origin2.shape[0:2]

        # Detect lines and normals
        #white_o = self.detector2.detectLines('white')
        yellow_o2 = self.detector2.detectLines('yellow')
        #hough_threshold = 20
        #hough_min_line_length = 1
        #hough_max_line_gap = 1
        red_o2 = self.detector2.detectLines('red')

        #################################################3

        if len(white.lines) > 0 or len(yellow.lines) > 0 or len(red.lines) > 0 :
            self.Linedisable = False

        if len(red_o.lines) > 0:
            print("red siganl detect !!!!!!!!!!!!!!!!!!!")
            self.pub_signal.publish("RED")

        #if len(yellow.lines) < 1 :
            #print("yellow ......... disable ")

        # SegmentList constructor
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp

        # Yellow, White Line add segments to segmentList
        if len(white.lines) > 0:
            segmentList.segments.extend(self.toSegmentMsg(white.lines, white.normals, Segment.WHITE))
        if len(yellow.lines) > 0:
           segmentList.segments.extend(self.toSegmentMsg(yellow.lines, yellow.normals, Segment.YELLOW))
        #if len(red.lines) > 0:
        #   segmentList.segments.extend(self.toSegmentMsg(red.lines, red.normals, Segment.RED))
        # Publish segmentList
        self.pub_lines.publish(segmentList)

        # Draw lines and normals
        image_with_lines = np.copy(image_cv_corr)
        drawLines(image_with_lines, white.lines, (0, 0, 0))
        drawLines(image_with_lines, yellow.lines, (255, 0, 0))

        image_with_lines_o = np.copy(image_cv_corr_origin)
        drawLines(image_with_lines_o, red_o.lines, (0, 255, 0))

        #image_with_lines_o2 = np.copy(image_cv_corr_origin2)
        #drawLines(image_with_lines_o, white.lines, (0, 0, 0))
        #drawLines(image_with_lines_o2, yellow_o2.lines, (255, 0, 0))
        #drawLines(image_with_lines_o2, red_o2.lines, (0, 255, 0))

        # Publish the frame with lines
        image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
        image_msg_out.header.stamp = image_msg.header.stamp
        self.pub_image_with_line.publish(image_msg_out)

        # Publish the frame with lines
        image_msg_out_o = self.bridge.cv2_to_imgmsg(image_with_lines_o, "bgr8")
        self.pub_image_with_line_o.publish(image_msg_out_o)

        # Publish the frame with lines
        #image_msg_out_o2 = self.bridge.cv2_to_imgmsg(image_with_lines_o2, "bgr8")
        #image_msg_out_o.header.stamp = image_msg.header.stamp
        #self.pub_image_with_line_o2.publish(image_msg_out_o2)

    def onShutdown(self):
        self.loginfo("Shutdown.")
        self.sub_image.unregister()
        self.sub_motor_start.unregister()
        #self.Timer.shutdown()

    def toSegmentMsg(self,  lines, normals, color):

        segmentMsgList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
            segmentMsgList.append(segment)
        return segmentMsgList

class Stats():
    def __init__(self):
        self.nresets = 0
        self.reset()

    def reset(self):
        self.nresets += 1
        self.t0 = time.time()
        self.nreceived = 0
        self.nskipped = 0
        self.nprocessed = 0

    def received(self):
        if self.nreceived == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node received first image.')
        self.nreceived += 1

    def skipped(self):
        self.nskipped += 1

    def processed(self):
        if self.nprocessed == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node processing first image.')

        self.nprocessed += 1

    def info(self):
        delta = time.time() - self.t0

        if self.nreceived:
            skipped_perc = (100.0 * self.nskipped / self.nreceived)
        else:
            skipped_perc = 0

        def fps(x):
            return '%.1f fps' % (x / delta)

        m = ('In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' %
             (delta, self.nreceived, fps(self.nreceived),
              self.nprocessed, fps(self.nprocessed),
              self.nskipped, fps(self.nskipped), skipped_perc))
        return m





if __name__ == '__main__':
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
rospy.spin()