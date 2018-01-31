from abc import ABCMeta, abstractmethod
from collections import namedtuple

Detections = namedtuple('Detections', 'lines normals area centers')

class LineDetectorInterface():
    __metaclass__ = ABCMeta


    @abstractmethod
    def setImage(self, bgr):
        pass

    @abstractmethod
    def detectLines(self, color):
        """ Returns a tuple of class Detections """
    def detectLines2(self, color, hough_threshold, hough_min_line_length , hough_max_line_gap):
        """ Returns a tuple of class Detections """


