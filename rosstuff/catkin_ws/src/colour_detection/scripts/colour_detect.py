#! /usr/bin/env python3.8
import rospy
from std_msgs.msg import Int16MultiArray, Int16
from sensor_msgs.msg import Image 
from fiducial_msgs.msg import FiducialArray, FiducialTransformArray
import numpy as np
from time import sleep

"""
Color detector using OpenCV and Ximea Camera
Author: Miguel Valencia
"""
import cv2
import os
from math import *
from numpy.core.fromnumeric import argmin, argmax

'''
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
'''

MOUSE_LEFT = 1
class ColorDetector:

    """
    Color detection class:
    Detects RGBY as well as White (W) and Black (K)
    """

    RED     = 0
    GREEN   = 1
    BLUE    = 2
    YELLOW  = 3
    WHITE   = 4
    BLACK   = 5

    def __init__(self, bgr_r, bgr_g, bgr_b, bgr_y):
        self.bgr_r = bgr_r
        self.bgr_g = bgr_g
        self.bgr_b = bgr_b
        self.bgr_y = bgr_y

        # Convert to hsv color space (H = Hue, S = Saturation, V = Value)
        self.hsv_r = ColorDetector.bgr2hsv(self.bgr_r)
        self.hsv_g = ColorDetector.bgr2hsv(self.bgr_g)
        self.hsv_b = ColorDetector.bgr2hsv(self.bgr_b)
        self.hsv_y = ColorDetector.bgr2hsv(self.bgr_y)

    @classmethod
    def bgr2hsv(cls, bgr):
        """
        If only there was a nicer way to convert single triples of BGR to HSV
        """
        return cv2.cvtColor(np.array([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]

    @classmethod
    def hsv2bgr(cls, hsv):
        """
        If only there was a nicer way to convert single triples of HSV to BGR
        """
        return cv2.cvtColor(np.array([[hsv]]), cv2.COLOR_HSV2BGR)[0][0]

    @classmethod
    def hsv2coord(cls, hsv):
        """
        Maps to a cone with max radius = 1, height = 1
        This representation is singularity free compared to the HSV space
        """
        # Hue is normalised from 0 to 2 pi
        h = 2 * pi * hsv[0] / 180
        # Saturation is normalised from zero to 
        s = hsv[1] / 255
        v = hsv[2] / 255
        return np.array([v * s * cos(h), v * s * sin(h), v])

    def detect_color(self, bgr):
        """
        Color detection implementation  
        Maps the HSV space as a cone, and finds the cartesian distance
        The closest distance color to the test color is the color returned.
        Black and white are also added to this color detection
        """
        hsv = ColorDetector.bgr2hsv(bgr)
        coord_test = ColorDetector.hsv2coord(hsv)
        coord_r = ColorDetector.hsv2coord(self.hsv_r)
        coord_g = ColorDetector.hsv2coord(self.hsv_g)
        coord_b = ColorDetector.hsv2coord(self.hsv_b)
        coord_y = ColorDetector.hsv2coord(self.hsv_y)
        coord_w = ColorDetector.hsv2coord(np.array([0, 0, 255]))
        coord_k = ColorDetector.hsv2coord(np.array([0, 0, 0]))
        
        dist_r = np.linalg.norm(coord_test - coord_r)
        dist_g = np.linalg.norm(coord_test - coord_g)
        dist_b = np.linalg.norm(coord_test - coord_b)
        dist_y = np.linalg.norm(coord_test - coord_y)
        dist_w = np.linalg.norm(coord_test - coord_w)
        dist_k = np.linalg.norm(coord_test - coord_k)
        dist_list = [dist_r, dist_g, dist_b, dist_y, dist_w, dist_k]
        if min(dist_list) < 2:
            return argmin(dist_list)
        else:
            return 5




def demo(rgb_data):
    """
    This is demo code for color detection.
    You will need to modify this for your own implemention to use ROS.
    It is suggested to use a subscriber/publisher node to listen for Images, and publish the color detected.
    """
    # Read the colors.config file from each line and set the color arrays
    current_dir = r"/home/metr4202/Documents/team_repo/metr4202_T10/rosstuff/catkin_ws/src/colour_detection/scripts"
    try:
        file = open(current_dir + r"/colors.config", 'r')
        i = 0
        for line in file:
            entries = line.split(' ')
            values = [int(x) for x in entries]
            if i == 0:
                bgr_r = np.array([values[0], values[1], values[2]]).astype(np.uint8)
            if i == 1:
                bgr_g = np.array([values[0], values[1], values[2]]).astype(np.uint8)
            if i == 2:
                bgr_b = np.array([values[0], values[1], values[2]]).astype(np.uint8)
            if i == 3:
                bgr_y = np.array([values[0], values[1], values[2]]).astype(np.uint8)
            i += 1
    except FileNotFoundError:
        print("Could not load color.config file")
        print("Setting to default values:")
        bgr_r = np.array([0, 0, 255]).astype(np.uint8)
        bgr_g = np.array([0, 255, 0]).astype(np.uint8)
        bgr_b = np.array([255, 0, 0]).astype(np.uint8)
        bgr_y = np.array([0, 255, 255]).astype(np.uint8)
    # Initialise the color detector
    color_detector = ColorDetector(bgr_r, bgr_g, bgr_b, bgr_y)
    c = color_detector.detect_color(np.array([rgb_data[0], rgb_data[1], rgb_data[2]]).astype(np.uint8))
    #print("thinks its this colour:",c)
    return c


class Coords():
    def __init__(self):
        self._msg = Int16MultiArray()
        #fiducial vertex in form [X0,Y0,X1,Y1,X2,Y2,X3,Y3]
        self._vertex = [0,0,0,0,0,0,0,0]
        self._height = 1024
        self._width = 1280 
        self._box_size = 20
        self._center_coords = [750, 550]
        self._ToScan = [0,0,0]
        self._current_index = 0
        self._image = []
        #publish second entry to michael_boss
        self._colour_pub = rospy.Publisher("/colour_data", Int16MultiArray, queue_size=1)
        #colour tallies 
        self._red = 0
        self._blue = 0
        self._yellow = 0
        self._green = 0
        #are we in the upright position
        self._colour_bin = 0

        #structure [XMax, Ymax, Xmin, Ymin]
        #self._vertex_zero_range = [0,0,0,0]
        #self._vertex_one_range = [0,0,0,0]
        #self._vertex_two_range = [0,0,0,0]
        #self._vertex_three_range = [0,0,0,0]
        #structure [vertex_max_min_one, ...]
        #self._all_boxes = [self._vertex_zero_range, self._vertex_one_range, self._vertex_two_range, self._vertex_three_range]
        self._list_of_bgr = []
        self._scanning_flag = False
        
    #def vertex_callback(self, data):
        #if self._scanning_flag:
     #       returnself._red, self._blue, self._green, self._yellow
        # self._vertex[1] = int(np.round(data.fiducials[0].y0))
        # self._vertex[2] = int(np.round(data.fiducials[0].x1))
        # self._vertex[3] = int(np.round(data.fiducials[0].y1))
        # self._vertex[4] = int(np.round(data.fiducials[0].x2))
        # self._vertex[5] = int(np.round(data.fiducials[0].y2))
        # self._vertex[6] = int(np.round(data.fiducials[0].x3))
        # self._vertex[7] = int(np.round(data.fiducials[0].y3))
        #self.make_vertex_boxes()
    
    def image_callback(self, data):
        if self._scanning_flag:
            return
        self._image = data.data
        # self._scanning_flag = True

    def colour_data(self, data):
        self._scan_bin = data
        print(data)
        if  self._scan_bin.data[0] == 1:
            self.scan_boxes()

    def get_bgr_from_coord(self,x,y, image):
        self._current_index = 3*(y-1)*self._width + 3*(x-1)

    def get_bgr(self):
        return self._bgr_send

    # def make_vertex_boxes(self):
    #     #populate the all_boxes arrayv
    #     vertexCoord = 0
    #     for range in self._all_boxes:    
            
    #         for coord in self._vertex:
    #             if vertexCoord%2 == 0:
    #                 #this is an x coordinate (even numbers)
    #                 range[0] = int(self._vertex[vertexCoord] + self._box_size/2)
    #                 range[2] = int(self._vertex[vertexCoord] - self._box_size/2)
    #                 vertexCoord+=1
    #             else:
    #                 #y coordinate
    #                 range[1] = int(se#what colour did we get form miguels code
    #    self._colour = 0f._vertex[vertexCoord] + self._box_size/2)
    #                 range[3] = int(self._vertex[vertexCoord] - self._box_size/2)
    #                 vertexCoord+=1
    #                 break
                

        
    #     self.scan_boxes()

    def scan_boxes(self):
        """
               _ ________________________
        x=1  |B|G|R|B|G|R|...B|G|R|B|G|R|
        y=1  |_____|_____|________|_____|
                    x=2   x=3      x=1280 x=1y=2
        """
        #generate list of coordinates to scan and then send to miguel's script
        while len(self._image) == 0:
            sleep(0.001)
            #wait for image data    
        
        self._scanning_flag = True
        #generate vertices
        cx = self._center_coords[0]
        cy = self._center_coords[1]
        self._vertex[0] = int(np.round(cx + self._box_size/2))
        self._vertex[1] = int(np.round(cy + self._box_size/2))
        self._vertex[2] = int(np.round(cx + self._box_size/2))
        self._vertex[3] = int(np.round(cy - self._box_size/2))
        self._vertex[4] = int(np.round(cx - self._box_size/2))
        self._vertex[5] = int(np.round(cy + self._box_size/2))
        self._vertex[6] = int(np.round(cx - self._box_size/2))
        self._vertex[7] = int(np.round(cy - self._box_size/2))
        
        # for vertex in self._all_boxes:
        #     #iterate over each x and y coord of current box
        for x in range(self._vertex[6], self._vertex[0]+1, 1):
            for y in range(self._vertex[7], self._vertex[1]+1,1):
                self.get_bgr_from_coord(x,y, self._image)
                self._bgr_send = [self._image[self._current_index], self._image[self._current_index +1], self._image[self._current_index+2]]
                #print("bgr:", self._bgr_send)
                currentColour = demo(self._bgr_send)
                if currentColour == 0:
                    self._red+=1
                elif currentColour == 2:
                    self._blue+=1
                elif currentColour == 3:
                    self._yellow+=1
                elif currentColour == 1:
                    self._green+=1
                else:
                    #probs back or white so ignore
                    continue
        print("done")
        print(self._red, self._blue, self._green, self._yellow)
        self._msg.data = [0,argmax([self._red, self._green,self._blue, self._yellow])]
        print("I sent this")
        self._colour_pub.publish(self._msg)
        self._red = 0
        self._green = 0
        self._blue = 0
        self._yellow = 0
        
        self._scanning_flag = False


if __name__ == "__main__":
    DataToSend = Coords()
    rospy.init_node("colourful", anonymous=True)
    rospy.Subscriber("/ximea_cam/image_raw", Image, DataToSend.image_callback, queue_size=1)
    rospy.Subscriber("/colour_data", Int16MultiArray, DataToSend.colour_data , queue_size=1)
    rospy.spin()

