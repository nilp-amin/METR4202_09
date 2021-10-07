#!/usr/bin/python

import rospy
import tf

from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_msgs.msg import Bool
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

import modern_robotics as mr
import numpy as np

# TODO: Check if the AruCo markers have stopped moving
# TODO: Check which AruCo marker transforms are valid (e.g. still on unpicked.)
# TODO: Check which AruCo marker is on the right side of the table
# TODO: Publish (x, y, z, theta, phi, psi) realtive to space frame to topic

class RobotVision():
    def __init__(self):
        rospy.init_node("scara_cv", anonymous=True)
        self.block_transform_pub = rospy.Publisher("block_transform", FiducialTransform, queue_size=10)
        self.block_ready_pub = rospy.Publisher("block_ready", Bool, queue_size=10)
        self.fid_transform_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback, queue_size=1)
        self.scara_home_sub = rospy.Subscriber("/scara_home", Bool, self.ready_callback, queue_size=1)
        self.listner = tf.TransformListener()
        self.rate = rospy.Rate(0.1)

        self.ready_msg = String() # Used to tell robot_kinematics that transform frame is a valid pickup location 
        self.tf_msg = FiducialTransform() # Used to send the transform frame of block
        # TODO: Get actual measurements and orientation for this transform
        self.Tbase_cam = np.array([
                                        [1, 0, 0, 0.1],
                                        [0, 1, 0, 0.0],
                                        [0, 0, 1, 0.5],
                                        [0, 0, 0, 1.0]
                                  ])
        self.old_distance = 0
        self.motion_stopped = False
        self.robot_ready = True 
        self.SCARA_ARM_RADIUS = 0.16
        pass

    def ready_callback(self, data):
        self.robot_ready = data.data
        pass

    def fiducial_callback(self, data):
        rospy.loginfo(data.transforms)
        try:
            """
            if len(data.transforms):
                rospy.loginfo(data.transforms[0].fiducial_id)
                x = data.transforms[0].transform.translation.x
                y = data.transforms[0].transform.translation.y
                z = data.transforms[0].transform.translation.z
                dist = [x, y, z]
                rospy.loginfo(self.marker_distance(dist))
            """
            if len(data.transforms) and self.robot_ready:
                (trans, rot)  = self.listner.lookupTransform("/0", 
                        f"/fiducial_{data.transforms[0].fiducial_id}", rospy.Time(0))
                # Check if a block is still moving
                if not self.is_moving(trans):
                    (fid_id, Tbase_block) = self.find_block_transform(data)
                    if fid_id is None or Tbase_block is None:
                        rospy.loginfo("All blocks found were out of reach")
                        return
                        pass
                    self.update_tf_msg(fid_id, Tbase_block)
                    self.block_transform_pub.publish(self.tf_msg)
                    self.tf_msg = FiducialTransform()

                    # Let scara_kinematics know that the transformations published are valid
                    bool_msg = Bool()
                    bool_msg.data = True
                    self.block_ready_pub.publish(bool_msg)
                    rospy.sleep(2)
                    bool_msg.data = False
                    self.block_ready_pub.publish(bool_msg)
                    pass
                pass
            else:
                bool_msg = Bool()
                bool_msg.data = False
                self.block_ready_pub.publish(bool_msg)
        except tf.LookupException as LE:
            rospy.loginfo(LE)
            pass
        except tf.ConnectivityException as CE:
            rospy.loginfo(CE)
            pass
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()
        pass

    def update_tf_msg(self, fid_id:int, transform:np.array):
        self.tf_msg.fiducial_id = fid_id
        (rot, p) = mr.TransToRp(transform)
        self.tf_msg.transform.translation.x = p[0]
        self.tf_msg.transform.translation.y = p[1]
        self.tf_msg.transform.translation.z = p[2]
        skew_omega = mr.MatrixLog3(rot)
        rotation_vec = mr.so3ToVec(skew_omega)
        self.tf_msg.transform.rotation.x = rotation_vec[0]
        self.tf_msg.transform.rotation.y = rotation_vec[1]
        self.tf_msg.transform.rotation.z = rotation_vec[2]
        pass

    def marker_distance(self, trans: list)->float:
        pos = np.array([trans[0], trans[1], trans[2]])
        return np.linalg.norm(pos)
        pass

    def is_moving(self, trans:list)->bool:
        if not self.motion_stopped:
            dist = self.marker_distance(trans)
            if dist == self.old_distance:
                self.motion_stopped = True
                result = True
            else:
                self.old_distance = dist
                result = False
        else:
            result = False

        rospy.sleep(1) # TODO: Might have to use a counter method instead of sleep
        return result
        pass

    def find_block_transform(self, data)->tuple:
        fiducials = {}
        max_dist_id = None
        max_dist = 0
        for tf in data.transforms:
            fid_id = tf.fiducial_id
            (trans, rot) = self.listner.lookupTransform("/0", f"/fiducial_{fid_id}", rospy.Time(0))
            Rcam_block = self.rotation_matrix(rot)
            Pcam_block = np.array([
                                    [trans[0]],
                                    [trans[1]],
                                    [trans[2]]
            ])
            Tcam_block = np.block([
                                    [Rcam_block , Pcam_block],
                                    [0, 0, 0, 1]
                                  ])
            Tbase_block = self.Tbase_cam @ Tcam_block
            block_posx = Tbase_block[0][-1]
            block_posy = Tbase_block[1][-1]
            # The height is always constant and does not affect choice of block
            curr_dist = self.marker_distance([block_posx, block_posy, 0])
            if curr_dist < self.SCARA_ARM_RADIUS:
                return (curr_dist, Tbase_block)
        return (None, None)
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
        pass

if __name__ == "__main__":
    try:
        rv = RobotVision()
        rv.run()
        pass
    except rospy.ROSInterruptException:
        pass
    pass
