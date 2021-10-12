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
        self.pub_angles = rospy.Publisher("/scara_angles", Float32MultiArray, queue_size=10)
        self.pub_move = rospy.Publisher("/scara_move", Bool, queue_size=10)
        self.sub_bt = rospy.Subscriber("/block_transform", FiducialTransform, self.transform_callback)
        self.sub_ready = rospy.Subscriber("/block_ready", Bool, self.ready_callback)

        #TODO: REMOVE THIS
        self.pub_test_blocktransform = rospy.Publisher("/block_transform", FiducialTransform, queue_size=10)

        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(1)

        #TODO: CHANGE THESE TO ACTUAL VALUES
        #Link Lengths
        self.l1 = 126.412e-3
        self.l2 = 67.5e-3
        #self.l1 = 20
        #self.l2 = 20

        #TODO: CHANGE BLOCK ID OPTIONS TO ACTUAL IDS
        self.block_id1 = 1
        self.block_id2 = 2
        self.block_id3 = 3
        self.block_id4 = 4
        
        self.robot_ready = False 

        #degrees
        self.rotation_limit = 114

    #TODO: REMOVE THIS
    def publish_test_values(self):
        T = Transform()
        T.translation.x = 0.1
        T.translation.y = 0.1
        T.translation.z = 0.1
        T.rotation.x = 0
        T.rotation.y = 0
        T.rotation.z = 43
        T.rotation.w = 10
        ft = FiducialTransform()
        ft.transform = T
        self.pub_test_blocktransform.publish(ft)
    
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
    #TODO: CHANGE TO ACTUAL VALUES
    def find_placement_angles(self, block_id):
        if(block_id == self.block_id1):
            return [-103.16, 15.162, 90]
        if(block_id == self.block_id2):
            return[-156.29918175, 15.16290002, 90]
        if(block_id == self.block_id3):
            return[-193.1690794, 15.16290002, 90]
        if(block_id == self.block_id4):
            return[-193.1690794, 15.16290002,90]
        
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
        theta2_1 = atan2(costheta2, sqrt(1 - costheta2**2 ))
        theta2_2 = atan2(costheta2, -sqrt(1 - costheta2**2 ))
        #theta1: First Link Rotation
        theta1_1 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_1), l2*sin(theta2_1))
        theta1_2 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_2), l2*sin(theta2_2))
        
        theta1,theta2 = self.choose_optimal_angle([theta1_1,theta2_1, theta1_2, theta2_2])

        #theta3: Rotation of End Effector
        explicit_quat = [r.x, r.y, r.z, r.w]
        euler_angles = tf.transformations.euler_from_quaternion(explicit_quat)
        theta3 = euler_angles[2]# find angle about z
        return [theta1, theta2, theta3]

    #Continuously being updated from subscriber
    def ready_callback(self, msg):
        self.robot_ready = msg.data
        
        #TODO: REMOVE THIS
        self.publish_test_values()
    
    #Continuously being checked
    def transform_callback(self, ft):
        if self.robot_ready:
            rospy.sleep(2)
            p = ft.transform.translation
            q = ft.transform.rotation
            block_id = ft.fiducial_id
            
            pickup_angles = self.compute_ik(p, q)
            placement_angles = self.find_placement_angles(block_id)

            msg = Float32MultiArray()
            msg.data = pickup_angles + placement_angles
            self.pub_angles.publish(msg)
            self.pub_move.publish(True)

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

        
