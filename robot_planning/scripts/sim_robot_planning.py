#!/usr/bin/python3

import rospy
from rospy.exceptions import ROSTimeMovedBackwardsException
import tf

from math import *
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

class RobotTrajectory():
    def __init__(self):
        rospy.init_node("scara_trajectory", anonymous=True)
        self.angle_sub = rospy.Subscriber("/scara_angles", Float32MultiArray, self.angle_callback, queue_size=1)
        self.joint1_pub = rospy.Publisher("/scara_simple/Rev123_position_controller/command", Float64, queue_size=1)
        self.joint2_pub = rospy.Publisher("/scara_simple/Rev126_position_controller/command", Float64, queue_size=1)
        self.joint3_pub = rospy.Publisher("/scara_simple/Rev110_position_controller/command", Float64, queue_size=1)
        pass

    def angle_callback(self, data):
        joint_1 = data.data[0]
        joint_2 = data.data[1]
        joint_3 = data.data[2]
        try:
            print("Trajectory generated")
            msg = Float64()
            msg.data = joint_1
            self.joint1_pub.publish(msg)
            msg.data = joint_2
            self.joint2_pub.publish(msg)
            msg.data = joint_3
            self.joint3_pub.publish(msg)
            pass
        except e:
            print(e)
            pass
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()
        pass

if __name__ == "__main__":
    try:
        rt = RobotTrajectory()
        rt.run()
    except rospy.ROSInterruptException:
        print("An error occurring running Robot Tracjectory.")
        pass