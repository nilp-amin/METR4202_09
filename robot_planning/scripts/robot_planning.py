#!/usr/bin/python

import rospy
import tf

from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_msgs.msg import Bool


import modern_robotics as mr
import numpy as np



class RobotTrajectory():
    def __init__(self):
        rospy.init_node("scara_trajectory", anonymous = True)
        self.pub = rospy.Publisher("desired_joint_states", queue_size=10)
        self.pub = rospy.Publisher("robot_ready", queue_size = 10)
        self.sub = rospy.Subscriber("scara_angles")
        self.sub = rospy.Subscriber("joint_states")
        self.rate = rospy.Rate(1)
