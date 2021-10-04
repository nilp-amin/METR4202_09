#!/usr/bin/python

import rospy
import tf

from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_msgs.msg import Bool
from fiducial_msgs.msg import FiducialTransformArray

import modern_robotics as mr
import numpy as np

# TODO: Check if the AruCo markers have stopped moving
# TODO: Check which AruCo marker transforms are valid (e.g. still on unpicked.)
# TODO: Check which AruCO marker is on the right side of the table
# TODO: Publish (x, y, z, theta, phi, psi) realtive to space frame to topic

class RobotVision():
    def __init__(self):
        rospy.init_node("scara_cv", anonymous=True)
        self.pub = rospy.Publisher("block_transform", FiducialTransformArray, queue_size=10)
        self.pub = rospy.Publisher("block_ready", String, queue_size=10)
        self.sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)
        self.sub = rospy.Subscriber("/scara_home", Bool, self.ready_callback)
        self.listner = tf.TransformListener()
        self.rate = rospy.Rate(1)

        self.ready_msg = String() # Used to tell robot_kinematics that transform frame is a valid pickup location 
        self.tf_msg = FiducialTransformArray() # Used to send the transform frame of block
        # TODO: Get actual measurements and orientation for this transform
        self.Tbase_cam = np.array([
                                        [1, 0, 0, 0.1],
                                        [0, 1, 0, 0.0],
                                        [0, 0, 1, 0.5].
                                        [0, 0, 0, 1.0]
                                  ])
        self.old_distance = 0
        self.motion_stopped = False
        self.robot_ready = False 
        pass

    def self.ready_callback(self, data):
        self.robot_ready = data.data
        pass

    def callback(self, data):
        # rospy.loginfo(data.transforms) # data.transforms -> list()
        if (len(data.transforms)) and self.robot_ready:
            (trans, rot)  = self.listner.lookupTransform("/0", 
                    f"/fiducial_{data.transforms[0].fiducial_id}", rospy.Time(0))
            # Check if a block is still moving
            if not is_moving(trans):
                (fid_id, trans, rot) = self.find_block_transform(data)
                self.update_tf_msg(fid_id, trans, rot)
                self.pub.publish(self.tf_msg)
                pass
            
            rospy.loginfo(data.transforms[0].fiducial_id)
            rospy.loginfo(dist)
            pass
        else:
            pass
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()
        pass

    def marker_distance(trans: list)->float:
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
        # TODO: Will change this algo to be less random when Tbase_cam is defined
            # It will check for the position of the block on the robot side of the rotating plate 
            # And return transforms for it if found
        for tf in data.transforms:
            fid_id = tf.fiducial_id
            (trans, rot)  = self.listner.lookupTransform("/0", 
                    f"/fiducial_{fid_id}", rospy.Time(0))
            curr_dist = marker_distance(trans)
            if curr_dist > max_dist:
                max_dist = curr_dist
                max_dist_id = fid_id
            pass
        (trans, rot)  = self.listner.lookupTransform("/0", 
                f"/fiducial_{max_dist_id}", rospy.Time(0))
        return (max_dist_id, trans, rot)
        pass

    def run2(self):
        rospy.sleep(3) # Sleep added so information is not lost when starting up 
        while not rospy.is_shutdown():
            try:
                # /0 is the camera
                (trans, rot)  = self.listner.lookupTransform("/0", "/fiducial_105", rospy.Time(0))
                rospy.loginfo(trans)
                rospy.loginfo(rot)
                pass
            except tf.LookupException as LE:
                rospy.loginfo(LE)
                pass
            except tf.ConnectivityException as CE:
                rospy.loginfo(CE)
                pass
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rv = RobotVision()
        rv.run()
        pass
    except rospy.ROSInterruptException:
        pass
    pass
