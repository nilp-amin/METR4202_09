#!/usr/bin/python3

import rospy
import tf
import modern_robotics as mr
import numpy as np

from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class Calibration():
    def __init__(self):
        rospy.init_node("calibration", anonymous=True)
        self.cam_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback, queue_size=1)
        self.listner = tf.TransformListener()
        self.rate = rospy.Rate(0.5)
        pass

    def rotation_matrix(self, rotation)->np.array:
        # Rotation in radians
        theta_x = rotation[0]
        theta_y = rotation[1]
        theta_z = rotation[2]

        rx = np.array([
                            [1, 0, 0],
                            [0, np.cos(theta_x), -np.sin(theta_x)],
                            [0, np.sin(theta_x), np.cos(theta_x)]
        ])
        ry = np.array([
                            [np.cos(theta_y), 0, np.sin(theta_y)],
                            [0, 1, 0],
                            [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])
        rz = np.array([
                            [np.cos(theta_z), -np.sin(theta_z), 0],
                            [np.sin(theta_z), np.cos(theta_z), 0],
                            [0, 0, 1]
        ])
        return rz @ ry @ rx

    def callback(self, data):
        print("no blocks found")
        if len(data.transforms) == 1:
            fid_id = data.transforms[0].fiducial_id
            (trans, rot) = self.listner.lookupTransform(f"fiducial_{5}", f"/fiducial_{4}", rospy.Time(0))
            euler_angles = tf.transformations.euler_from_quaternion(rot)
            Rbase_cam = self.rotation_matrix(euler_angles)
            Pbase_cam = np.array([
                                    [trans[0]],
                                    [trans[1]],
                                    [trans[2]]
            ])
            Tbase_cam = np.block([
                                    [Rbase_cam , Pbase_cam],
                                    [0, 0, 0, 1]
                                  ])
            print(Tbase_cam)
            self.rate.sleep()
            pass
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()



if __name__ == "__main__":
    try:
        c = Calibration()
        c.run()
        pass
    except:
        pass
