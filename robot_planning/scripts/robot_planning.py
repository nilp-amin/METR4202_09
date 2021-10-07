#!/usr/bin/python

import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

import modern_robotics as mr
import numpy as np

class RobotTrajectory():
    def __init__(self):
        rospy.init_node("scara_trajectory", anonymous = True)
        self.desired_joint_state_pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=10)
        self.scara_home_pub = rospy.Publisher("/scara_home", Bool, queue_size = 10)
        self.angle_sub = rospy.Subscriber("/scara_angles", Float32MultiArray, self.callback_angle)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.callback)
        self.allowable_sub = rospy.Subscriber("/scara_move", Bool, self.boolcall)
        self.rate = rospy.Rate(0.25)
        
        self.ready = False
        self.cv_ready = False
        self.home_config = [2, 2, 2, 2]
        self.current_joint_pos = []
        self.to_move_pos = []

    # Setting an empty array as the current joint
    # position, array given by the servos
    def callback(self, data):
        self.current_joint_pos = data.position
        if self.ready:
            to_move = np.array(self.to_move_pos)
            current_pos = np.array(self.current_joint_pos)
            error = to_move - current_pos
            if error[0] < 0.01 and error[1] < 0.01 and error[2] < 0.01 and error[3] < 0.01:
                

        self.rate.sleep()

    # Checking if the joint angles are
    # valid to be acted upon
    def boolcall(self, data):
        self.ready = data.data
        self.rate.sleep()

    # Setting an empty array as the array given by the
    # /scara_angles topic.
    # The method requires validation from /scara_move topic
    def callback_angle(self, data):
        if self.ready:
            self.to_move_pos = data.data

        self.rate.sleep()

    def run(self):
        rate.sleep(3)
        rospy.spin()

    if __name__ == '__main__':
