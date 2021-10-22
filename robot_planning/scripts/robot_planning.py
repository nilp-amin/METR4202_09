#!/usr/bin/python3

import rospy
import RPi.GPIO as GPIO
import math

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class RobotTrajectory():
    def __init__(self):
        rospy.init_node("scara_trajectory", anonymous=True)
        self.desired_joint_state_pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=1)
        self.scara_home_pub = rospy.Publisher("/scara_home", Bool, queue_size = 1)
        self.joint_sub = rospy.Subscriber("/scara_angles", Float32MultiArray, self.ik_joints_callback, queue_size=1)
        self.rate = rospy.Rate(0.25)

        servoPIN = 12
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPIN, GPIO.OUT)
        p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
        p.start(5) # Initialization

        self.pick_up_block = False
        self.pitch = 8e-3
        self.G = 8.11
        self.prismatic_lower_dist = 30e-3
        self.search_postion = [0, 2, 0, 0] 
        self.wait_time = 2
        self.prismatic_wait_time = 2

    def ik_joints_callback(self, joint_angles):
        print("Publishing joint states.")
        pickup_theta_1 = joint_angles.data[0]
        # pickup_theta_2 = self.prismatic_lower_dist / (self.pitch * self.G) 
        pickup_theta_2 = -2 
        pickup_theta_3 = joint_angles.data[1]
        pickup_theta_4 = joint_angles.data[2]

        dropoff_theta_1 = joint_angles.data[3]
        dropoff_theta_2 = 0
        dropoff_theta_3 = joint_angles.data[4]
        dropoff_theta_4 = joint_angles.data[5]

        # First publish that the robot is at search position 
        scara_home_msg = Bool()
        scara_home_msg.data = False 
        self.scara_home_pub.publish(scara_home_msg)

        # First we move directly above the block
        joint_msg = JointState()
        joint_msg.name = ["joint_1", "joint_3", "joint_4"]
        joint_msg.position = [pickup_theta_1, pickup_theta_3, pickup_theta_4]
        joint_msg.velocity = [1, 1, 1]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.wait_time)
        self.pick_up_block = True

        """
        # Now lower the gripper onto the block
        joint_msg.name = ["joint_2"]
        joint_msg.position = [pickup_theta_2]
        joint_msg.velocity = [2]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.prismatic_wait_time)
        """
        try:
            if self.pick_up_block == True:
                val = 5
                p.ChangeDutyCycle(val)
                print(val)
                time.sleep(2)
                self.pick_up_block = False
        except KeyboardInterrupt:
            p.stop()
            GPIO.cleanup()
        """
        # Move gripper above collision zone
        joint_msg.name = ["joint_2"]
        joint_msg.position = [dropoff_theta_2]
        joint_msg.velocity = [2]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.wait_time)
        """

        # Move robot to drop off zone
        joint_msg.name = ["joint_1", "joint_3", "joint_4"]
        joint_msg.position = [dropoff_theta_1, dropoff_theta_3, dropoff_theta_4]
        joint_msg.velocity = [1, 1, 1]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.wait_time)
        self.pick_up_block = True;

        try:
            if self.pick_up_block = True:
                val = 6.5
                p.ChangeDutyCycle(val)
                print(val)
                time.sleep(2)
                self.pick_up_block =
        except KeyboardInterrupt:
            p.stop()
            GPIO.cleanup()
        
        # Move robot back to search position
        joint_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
        joint_msg.position = self.search_postion 
        joint_msg.velocity = [1, 1, 1, 1]
        self.desired_joint_state_pub.publish(joint_msg)
        rospy.sleep(self.prismatic_wait_time)

        # publish that the robot is in search position
        scara_home_msg.data = True
        self.scara_home_pub.publish(scara_home_msg)
        pass

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        rt = RobotTrajectory()
        rt.run()
    except rospy.ROSInterruptionException:
        print("An error occurred running the Planning node.")
        pass
