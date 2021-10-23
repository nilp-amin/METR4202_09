#!/usr/bin/python3

"""
The ColorDetector class code was created 
by uqmvale6 (Miguel Valencia) and was cloned from
"git@github.com:uqmvale6/metr4202_color.git" 
"""
import rospy
import numpy as np
import os
import cv2

from sensor_msgs.msg import Image               # Obtaining RGB values at given pixel
from fiducial_msgs.msg import FiducialArray     # Obtaining vertices of fiducial
from std_msgs.msg import Int32                  # Obtaining fiducial id
from std_msgs.msg import String                 # Publishing colour of block

from math import *
from numpy.core.fromnumeric import argmin
from ximea import xiapi


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
        print(bgr)
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
        return argmin(dist_list)


class RobotColorDetect():
    def __init__(self, detector):
        rospy.init_node("colour_dectect", anonymous=True)
        self.colour_pub = rospy.Publisher("/block_colour", String, queue_size=1)
        self.fiducial_id_sub = rospy.Subscriber("/investigate_id", Int32, self.fiducial_id_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/ximea_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.fiducial_verticies_sub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.vertices_callback, queue_size=1)
        
        self.image_data = Image()
        self.vertices_data = FiducialArray()
        self.rate = rospy.Rate(10)
        self.height = 1024
        self.width = 1280
        self.detector = detector
        pass

    def get_bgr(self, x, y):
        b = self.image_data[(self.width) * (y - 1) + x]
        g = self.image_data[(self.width) * (y - 1) + (x + 1)]
        r = self.image_data[(self.width) * (y - 1) + (x + 2)]
        return np.array([b, g, r])
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()

    def find_fiducial_vertices(self, id):
        vertices_0 = []
        vertices_1 = []
        vertices_2 = []
        vertices_3 = []
        for vertices in self.vertices_data:
            if vertices.fiducial_id == id:
                vertices_0.append(int(vertices.x0))
                vertices_0.append(int(vertices.y0))
                vertices_1.append(int(vertices.x1))
                vertices_1.append(int(vertices.y1))
                vertices_2.append(int(vertices.x2))
                vertices_2.append(int(vertices.y2))
                vertices_3.append(int(vertices.x3))
                vertices_3.append(int(vertices.y3))
                break
        return (vertices_0, vertices_1, vertices_2, vertices_3)

    # Obtains fiducial id to investigate
    def fiducial_id_callback(self, data):
        id = data.data
        for i in range(1, 10):
            vertices_0, vertices_1, vertices_2, vertices_3 = self.find_fiducial_vertices(id)
            if len(vertices_0):
                print("could not find required block")
                print(f"trying again: {i}")
            else:
                break
        
        bgr = self.get_bgr(vertices_0[0] - 6, vertices_0[1]).astype(np.uint8)
        colour = ["red", "green", "blue", "yellow", "white", "black"][self.detector.detect_color(bgr)]
        print(colour)
        pass

    # Obtains rgb data of all camera pixels
    def image_callback(self, data):
        self.image_data = data.data
        self.rate.sleep()
        pass

    # Obtains the verticies of visible fiducials
    def vertices_callback(self, data):
        self.vertices_data = data.fiducials
        self.rate.sleep()
        pass

if __name__ == "__main__":
    try:
        # Read the colors.config file from each line and set the color arrays
        current_dir = os.getcwd()
        try:
            file = open(current_dir + "/colors.config", 'r')
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
        colour_detector = ColorDetector(bgr_r, bgr_g, bgr_b, bgr_y)
        rcd = RobotColorDetect(colour_detector)
        rcd.run()
    except rospy.ROSInterruptException:
        print("Colour detection node had an issue running.")
    pass