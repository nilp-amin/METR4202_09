#!/usr/bin/python

import rospy
import tf

from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray

import modern_robotics as mr

# TODO: Check if the AruCo markers have stopped moving
# TODO: Check which AruCo marker transforms are valid (e.g. still on unpicked.)
# TODO: Check which AruCO marker is on the right side of the table
# TODO: Publish (x, y, z, theta, phi, psi) realtive to space frame to topic

class RobotVision():
    def __init__(self):
        rospy.init_node("scara_cv", anonymous=True)
        self.pub = rospy.Publisher("block_transform", TFMessage, queue_size=10)
        self.pub = rospy.Publisher("block_ready", String, queue_size=10)
        self.sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)
        self.listner = tf.TransformListener()
        self.rate = rospy.Rate(1)

        self.ready_msg = String() # Used to tell robot_kinematics that transform frame is a valid pickup location 
        self.tf_msg = TFMessage() # Used to send the transform frame of block
        pass


    def callback(self, data):
        # rospy.loginfo(data.transforms) # data.transforms -> list()
        if (len(data.transforms)):
            (trans, rot)  = self.listner.lookupTransform("/0", f"/fiducial_{data.transforms[0].fiducial_id}", rospy.Time(0))
            rospy.loginfo(trans)
            rospy.loginfo(rot)
            pass
        pass

    def run(self):
        rospy.sleep(3)
        rospy.spin()
        pass


    # TODO: Could use /fiducial_transform to check which transforms are valid
    # TODO: then use lookupTransform to transform the frames for us
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
