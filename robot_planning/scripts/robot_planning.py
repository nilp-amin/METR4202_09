#!/usr/bin/python3

import rospy
import gpiozero
import math

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class RobotTrajectory():
    def __init__(self):
        rospy.init_node("scara_trajectory", anonymous=True)
        self.desired_joint_state_pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=1)
        self.scara_home_pub = rospy.Publisher("/scara_home", Bool, queue_size = 1)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.ik_joints_callback, queue_size=1)
        self.rate = rospy.Rate(0.25)

        self.pitch = 8e-3
        self.G = 8.11
        self.prismatic_lower_dist = 30e-3
        self.search_postion = [math.pi/2, 0, 0, 0] 
        self.wait_time = 2 

    def ik_joints_callback(self, joint_angles):
        pickup_theta_1 = joint_angles[0]
        pickup_theta_2 = self.prismatic_lower_dist / (self.pitch * self.G) 
        pickup_theta_3 = joint_angles[1]
        pickup_theta_4 = joint_angles[2]

        dropoff_theta_1 = joint_angles[3]
        dropoff_theta_2 = 0
        dropoff_theta_3 = joint_angles[4]
        dropoff_theta_4 = joint_angles[5]

        # First publish that the robot is at search position 
        scara_home_msg = Bool()
        scara_home_msg.data = False 
        self.scara_home_pub.publish(scara_home_msg)

        # First we move directly above the block
        joint_msg = JointState()
        joint_msg.name = ["joint_1", "joint_3", "joint_4"]
        joint_msg.position = [pickup_theta_1, pickup_theta_3, pickup_theta_4]
        joint_msg.velocity = [2, 2, 2]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.wait_time)

        # Now lower the gripper onto the block
        joint_msg.name = ["joint_2"]
        joint_msg.position = [pickup_theta_2]
        joint_msg.velocity = [1]
        rospy.sleep(self.wait_time)

        # TODO: Grab the block

        # Move gripper above collision zone
        joint_msg.name = ["joint_2"]
        joint_msg.position = [dropoff_theta_2]
        joint_msg.velocity = [1]
        rospy.sleep(self.wait_time)

        # Move robot to drop off zone
        joint_msg.name = ["joint_1", "joint_3", "joint_4"]
        joint_msg.position = [dropoff_theta_1, dropoff_theta_3, dropoff_theta_4]
        joint_msg.velocity = [2, 2, 2]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.wait_time)

        # TODO: Drop the block 

        # Move robot back to search position
        joint_msg.name = ["joint_1", "joint_3", "joint_4"]
        joint_msg.position = self.search_postion 
        joint_msg.velocity = [2, 2, 2]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.wait_time)

        # publish that the robot is in search position
        scara_home_msg.data = True
        self.scara_home_pub.publish(scara_home_msg)
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()


if __name__ == "__main__":
    try:
        rt = RobotTrajectory()
        rt.run()
    except rospy.ROSInterruptionException:
        print("An error occurred running the Planning node.")
        pass