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
        self.block_id1 = 0 # 1
        self.block_id2 = 2
        self.block_id3 = 3
        self.block_id4 = 4

        #PLACEMENTS
        #           1
        #   2
        #
        #   3
        #           4
        self.dropoff1 = [1.467, -1.306, radians(90)]
        self.dropoff2 = [2.395, -1.306, radians(90)]
        self.dropoff3 = [-2.395, 1.306, radians(90)]
        self.dropoff4 = [-1.467, 1.306, radians(90)]

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
         
        return [angles[0], angles[1]]


    def find_placement_angles(self, block_id):
        if(block_id == self.block_id1):
            return  self.dropoff1
        if(block_id == self.block_id2):
            return self.dropoff2
        if(block_id == self.block_id3):
            return self.dropoff3
        if(block_id == self.block_id4):
            return self.dropoff4
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
        
        #From lecture slides theta1->theta1 theta2->theta3
        #theta2: Second Link Rotation
        costheta3 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
        if (abs(costheta3) > 1): print("No solution could be found")

        theta3_1 = atan2(sqrt(1 - costheta3**2 ), costheta3)
        theta3_2 = atan2(-sqrt(1 - costheta3**2 ), costheta3)
        #theta1: First Link Rotation
        theta1_1 = atan2(p_y, p_x) - atan2(l2*sin(theta3_1), l1 + l2*cos(theta3_1))
        theta1_2 = atan2(p_y,p_x) - atan2(l2*sin(theta3_2), l1 + l2*cos(theta3_2))
        
        theta1,theta3 = self.choose_optimal_angle([theta1_1, theta3_1, theta1_2, theta3_2])

        #theta3: Rotation of End Effector
        theta4 = (r.z - theta3 - theta1) % radians(90)
        
        thetaarray = [theta1, -theta3, -theta4]
        return thetaarray
    
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

        
