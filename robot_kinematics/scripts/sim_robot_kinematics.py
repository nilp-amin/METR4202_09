#!/usr/bin/python3


import rospy
import tf

from math import *
from std_msgs.msg import Float32MultiArray
from fiducial_msgs.msg import FiducialTransform


class ComputeIk():
    def __init__(self):
        rospy.init_node("scara_sim_ik", anonymous=True)
        self.pub_angles = rospy.Publisher("scara_angles", Float32MultiArray, queue_size=1)
        self.block_transform = rospy.Subscriber("/block_transform", FiducialTransform, self.transform_callback, queue_size=1)
        self.l1 = 126.412e-3
        self.l2 = 67.5e-3
        self.rotation_limit = 114
        self.block_id1 = 1
        self.block_id2 = 2
        self.block_id3 = 3
        self.block_id4 = 4
        pass

    def find_placement_angles(self, block_id):
        if(block_id == self.block_id1):
            return [radians(-103.16), radians(15.162), radians(90)]
        if(block_id == self.block_id2):
            return[radians(-156.29918175), radians(15.16290002), radians(90)]
        if(block_id == self.block_id3):
            return[radians(-193.1690794), radians(15.16290002), radians(90)]
        if(block_id == self.block_id4):
            return[radians(-193.1690794), radians(15.16290002), radians(90)]
        
        #ERROR
        return[0,0,0]

    def meets_rotation_limit(self,  angle):
        angle = degrees(angle)
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

    def compute_ik(self, t, r):
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
        pass

    def transform_callback(self, data):
        try:
            print("robot kinematics published")
            p = data.transform.translation
            q = data.transform.rotation
            block_id = data.fiducial_id

            pickup_angles = self.compute_ik(p, q)
            placement_angles = self.find_placement_angles(block_id)

            msg = Float32MultiArray()
            msg.data = pickup_angles + placement_angles
            self.pub_angles.publish(msg)
        except e:
            print(e)
            print("robot kinematics still not ready")
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()

if __name__ == "__main__":
    try:
        ik = ComputeIk() 
        ik.run()
    except rospy.ROSInterruptException:
        print("An error occurred running the IK node.")
    pass