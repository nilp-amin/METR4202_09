#!/usr/bin/python

import rospy
import rosp


class RobotVision():
    def __init__(self):
        rospy.init_node("scara_cv", anonymous=True)
        self.rate = rospy.Rate(1)
        pass

    def run(self):
        sleep(3) # Sleep added so information is not lost when starting up 
        while not rospy.is_shutdon():
            rospy.loginfo("Hello world")
            self.rate.sleep()
            pass
        pass

if __name__ == "__main__":
    try:
        rv = RobotVision()
        rv.run()
        pass
    except rospy.ROSInterruptException:
        pass
    pass
