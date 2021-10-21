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

    def __init__(self):
        rospy.init_node("scara_kinematics", anonymous=True)
        self.pub_angles = rospy.Publisher("/scara_angles", Float32MultiArray, queue_size=1)
        self.sub_bt = rospy.Subscriber("/block_transform", FiducialTransform, self.transform_callback, queue_size=1)

        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1)

        #Link Lengths
        self.l1 = 125e-3
        self.l2 = 93e-3

        #TODO: CHANGE BLOCK ID OPTIONS TO ACTUAL IDS
        self.block_id1 = 1
        self.block_id2 = 2
        self.block_id3 = 3
        self.block_id4 = 4

        #degrees
        self.rotation_limit = 114

    def meets_rotation_limit(self,angle):
        angle = angle * (180/pi)
        if(angle > 0):
            if(angle > self.rotation_limit):
                return False
        if(angle < 0):
            if(angle < -self.rotation_limit):
                return False
        return True

    def choose_optimal_angle(self, angles):
         if(self.meets_rotation_limit(angles[0])):
                if(self.meets_rotation_limit(angles[1])):
                    return [angles[0], angles[1]]
         if(self.meets_rotation_limit(angles[2])):
                if(self.meets_rotation_limit(angles[3])):
                    return [angles[2], angles[3]]


    # TODO: (nilp) make sure you convert everything back to radians
    # use radians() and degrees() function instead of hard coding
    def find_placement_angles(self, block_id):
        if(block_id == self.block_id1):
            return [-103.16*(pi/180), 15.162*(pi/180), 90*(pi/180)]
        if(block_id == self.block_id2):
            return [-156.299*(pi/180), 15.162*(pi/180), 90*(pi/180)]
        if(block_id == self.block_id3):
            return [166.831*(pi/180), 15.162*(pi/180), 90*(pi/180)]
        if(block_id == self.block_id4):
            return [113.70*(pi/180), 15.162*(pi/180), 90*(pi/180)]
        
        #ERROR
        return[0,0,0]

    #where t and r are msg objects
    def compute_ik(self, t, r):
        # Initialization
        p_x = t.x
        p_y = t.y
        p_z = t.z
        l1 = self.l1
        l2 = self.l2

        #theta2: Second Link Rotation
        costheta2 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
        if (abs(costheta2) > 1): print("No solution could be found")

        theta2_1 = atan2(sqrt(1 - costheta2**2 ), costheta2)
        theta2_2 = atan2(costheta2, -sqrt(1 - costheta2**2 ))
        #theta1: First Link Rotation
        theta1_1 = atan2(p_y, p_x) - atan2(l2*sin(theta2_1), l1 + l2*cos(theta2_1))
        theta1_2 = atan2(p_y,p_x) - atan2(l2*sin(theta2_2), l1 + l2*cos(theta2_2))
        
        theta1,theta2 = self.choose_optimal_angle([theta1_1,theta2_1, theta1_2, theta2_2])

        #theta3: Rotation of End Effector
        theta3 = r.z
        return [theta1, -theta2, -theta3]
    
    #Continuously being checked
    def transform_callback(self, ft):
        rospy.sleep(2)
        p = ft.transform.translation
        q = ft.transform.rotation
        block_id = ft.fiducial_id
        
        pickup_angles = self.compute_ik(p, q)
        placement_angles = self.find_placement_angles(block_id)

        msg = Float32MultiArray()
        msg.data = pickup_angles + placement_angles
        print(msg.data)
        self.pub_angles.publish(msg)

    def run(self):
        rospy.sleep(3)
        rospy.spin()

if __name__ == '__main__':

    try:
        ik = ComputeIk()
        ik.run()
    except rospy.ROSInterruptException:
        print("An error occurred running the IK node.")
        pass

        
