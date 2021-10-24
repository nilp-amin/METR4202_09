#!/usr/bin/python3

import rospy
import tf

import numpy as np
from math import *
import cmath

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from fiducial_msgs.msg import FiducialTransform


class ComputeIk():
    """
    A class to represent the calculation of angles from given transform

    Atributes:
        pub_angles: publisher to /scara_angles topic
        sub_bt: subscriber to /block_transform topic
        rate: rate for operation within rospy
        l1: Link 1 length
        l2: Link 2 length
        block_id1: colour ID of block
        block_id2: colour ID of block
        block_id3: colour ID of block
        block_id4: colour ID of block
        dropoff1: Drop off location for zone 1
        dropoff2: Drop off location for zone 2
        dropoff3: Drop off location for zone 3
        dropoff4: Drop off location for zone 4
        rotation_limit: rotation limit of servos
    Methods:
        meets_rotation_limit(): Check if given angle is within the specified limits
        chose_optimal_angle(): Picks optimal angles depending on limits
        find_placement_angles(): Determines joint angles for desired zone
        compute_ik(): Use inverse kinematics to calculate joint angles from translations and rotation of block
        transform_callback(): callback for when angles are published to /block_transform topic
        run(): Runs spin() with a small sleep before hand to account for any errors
    """    
    def __init__(self):
        """
        Constructs all the necessary attributes for ComputeIk object
        """    
        # Initialise communication
        rospy.init_node("scara_kinematics", anonymous=True)
        self.pub_angles = rospy.Publisher("/scara_angles", Float32MultiArray, queue_size=1)
        self.sub_bt = rospy.Subscriber("/block_transform", FiducialTransform, self.transform_callback, queue_size=1)

        self.rate = rospy.Rate(2)

        #Link Lengths
        self.l1 = 125e-3
        self.l2 = 93e-3

        # ID CONVERSION
        # Red =  0
        # Green = 1
        # Blue = 2
        # Yellow = 3

        #PLACEMENTS
        #           1
        #   2
        #
        #   3
        #           4

        #Assign block ID locations
        # Compare values to 
        # Each number equals a specific colour
        # REORDER ON DAY TO MATCH GIVEN COLOUR DROP OFF LOCATIONS
        self.block_id1 = 0 # Location 1 drop off point
        self.block_id2 = 1 # Location 2 drop off point
        self.block_id3 = 2 # Location 3 drop off point
        self.block_id4 = 3 # Locaiton 4 drop off point
        self.dropoff1 = [1.467, -1.306, radians(90)]
        self.dropoff2 = [2.395, -1.306, radians(90)]
        self.dropoff3 = [-2.395, 1.306, radians(90)]
        self.dropoff4 = [-1.467, 1.306, radians(90)]

        # Set rotation limit
        self.rotation_limit = 114

    def meets_rotation_limit(self, angle):
        """
        Check if given angle is within the specified limits

        Args:
            angle (float): angle to check against limits

        Returns:
            True (Bool): if meets limit
            False (Bool): doesn't meet limits  
        """
        angle = degrees(angle)
        if(angle > 0):
            if(angle > self.rotation_limit):
                return False
        if(angle < 0):
            if(angle < -self.rotation_limit):
                return False
        return True
    
    def choose_optimal_angle(self, angles):
        """
        Function that picks optimal angles depending on limits if not return first set of angles in list

        Args:
            angles (list of floats): Two sets of potental angles to move to

        Returns:
            [angles[], angles[]]: optimal angles that take into account rotation limits

        """ 
        if(self.meets_rotation_limit(angles[0])):
                if(self.meets_rotation_limit(angles[1])):
                    return [angles[0], angles[1]]
        if(self.meets_rotation_limit(angles[2])):
                if(self.meets_rotation_limit(angles[3])):
                    return [angles[2], angles[3]]
         
        return [angles[0], angles[1]]


    def find_placement_angles(self, block_id):
        """
        Determines which angles to use for the desired drop off location from the specified block ID

        Args:
            block_id (int): ID of block for sorting

        Returns:
            [angle, angle, angle]: list of desired joint angles for drop off location
        """   
        if(block_id == self.block_id1):
            return  self.dropoff1
        if(block_id == self.block_id2):
            return self.dropoff2
        if(block_id == self.block_id3):
            return self.dropoff3
        if(block_id == self.block_id4):
            return self.dropoff4
        #ERROR
        return [0,0,0]

    #where t and r are msg objects
    def compute_ik(self, t, r):
        """ Given translation and rotation messages, calculate and return three respective joint angles

        Args:
            t (geometry_msgs/Vector3): Translation - containing x, y and z positions
            r (geometry_msgs/Quaternion: Rotation -  Represents an orientation in free space in quaternion form.

        Returns:
            thetaarray: Array of joint angles
        """
        # Initialization
        p_x = t.x
        p_y = t.y
        p_z = t.z
        l1 = self.l1
        l2 = self.l2
        
        #From lecture slides theta1->theta1 theta2->theta3
        # theta3: Second Link Rotation
        costheta3 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
        theta3_1 = atan2(sqrt(1 - costheta3**2 ), costheta3)
        theta3_2 = atan2(-sqrt(1 - costheta3**2 ), costheta3)
        # theta1: Link rotation
        theta1_1 = atan2(p_y, p_x) - atan2(l2*sin(theta3_1), l1 + l2*cos(theta3_1))
        theta1_2 = atan2(p_y,p_x) - atan2(l2*sin(theta3_2), l1 + l2*cos(theta3_2))
        # Find optimal angles from two sets calculated
        theta1, theta3 = self.choose_optimal_angle([theta1_1, theta3_1, theta1_2, theta3_2])
        # theta4: Rotation of End Effector
        theta4 = (r.z - theta3 - theta1) % radians(90)
        # Theta 3 and Theta 4 flipped due to orientation of servo
        theta_array = [theta1, -theta3, -theta4]
        return theta_array
    

    def transform_callback(self, ft):
        """
        Function that is called when data is published to /block_node
        Finds pickup angles from given FiducialTransform message 
        Publishes pickup angles and determined placement angles to /scara_angles
        as 6 angles (theta1, theta3, theta4, theta1, theta3, theta4)
            where first three are joint angles for picking up block and last three for 
            placing block

        Args:
            ft (fiducial_msgs/FiducialTransform): fiducial transform from /block_node topic
        """
        # Wait to remove likelihood of errors
        rospy.sleep(2)
        #Extract data from fiducial transform message    
        p = ft.transform.translation
        r = ft.transform.rotation
        block_id = ft.fiducial_id
        # Run inverse kinematics function
        pickup_angles = self.compute_ik(p, r)
        # Find where to place the block
        placement_angles = self.find_placement_angles(block_id)
        # Create message for publishing
        msg = Float32MultiArray()
        msg.data = pickup_angles + placement_angles
        self.pub_angles.publish(msg)
        

    def run(self):
        """
        Runs spin() with a small sleep before hand to account for any errors
        """
        rospy.sleep(3)
        #Run
        rospy.spin()
        

if __name__ == '__main__':

    try:
        ik = ComputeIk()
        ik.run()
    except rospy.ROSInterruptException:
        print("An error occurred running the IK node.")
        pass

        
