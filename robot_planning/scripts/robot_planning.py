#!/usr/bin/python

import rospy
import tf

from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
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
        self.rate = rospy.Rate(1)
        
        self.ready = False
        self.current_joint_pos = []

    def callback(self, data):
        self.current_joint_pos = data.position
        self.rate.sleep()

    def boolcall(self, data):
        self.ready = data.data
        self.rate.sleep()

    def callback_angles(self, data):
        if self.ready:
