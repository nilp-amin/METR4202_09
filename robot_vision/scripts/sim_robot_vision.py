#!/usr/bin/python3

import rospy
import tf
import numpy as np
import modern_robotics as mr

from std_msgs.msg import String
from std_msgs.msg import Bool
from fiducial_msgs.msg import FiducialTransform

class RobotVision():
    def __init__(self):
        rospy.init_node("scara_sim_cv", anonymous=True)
        self.block_transform_pub = rospy.Publisher("block_transform", FiducialTransform, queue_size=10)
        self.scara_home_sub = rospy.Subscriber("/scara_home", Bool, self.ready_callback, queue_size=1)
        self.Tbase_block = np.array([
            [1, 0, 0, 0.16],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        pass

    def ready_callback(self, data):
        try:
            if data.data:
                print("robot is ready")
                tf_msg = FiducialTransform()
                (rot, p) = mr.TransToRp(self.Tbase_block)
                tf_msg.fiducial_id = 1
                tf_msg.transform.translation.x = p[0]
                tf_msg.transform.translation.y = p[1]
                tf_msg.transform.translation.z = p[2]
                skew_omega = mr.MatrixLog3(rot)
                rotation_vec = mr.so3ToVec(skew_omega)
                tf_msg.transform.rotation.x = rotation_vec[0]
                tf_msg.transform.rotation.y = rotation_vec[1]
                tf_msg.transform.rotation.z = rotation_vec[2]
                tf_msg.transform.rotation.w = 0 
                ready_msg = Bool()
                ready_msg.data = True
                self.block_transform_pub.publish(tf_msg)
                pass
            else:
                print("robot is not ready")
        except:
            pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()

if __name__ == "__main__":
    try:
        rv = RobotVision()
        rv.run()
    except rospy.ROSInterruptException:
        pass
    pass