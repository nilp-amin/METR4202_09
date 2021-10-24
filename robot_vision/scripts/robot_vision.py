#!/usr/bin/python3

import rospy
import tf

import numpy as np
import modern_robotics as mr

from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class RobotVision():
    
    """
    A class to represent the detection of blocks using a camera

    Attributes:
        block_transform_pub: publisher to block_transform topic
        find_colour_pub: publisher to id topic
        scara_home_sub: subscriber to /scara_home topic
        fid_transform_sub: subscriber to /fiducial_transform topic
        colour_sub: subscriber to /block_colour topic
        listner: TODO
        tf_msg: FiducialTransorm() message init
        rate: rate for operation within rospy
        Tbase_fiducial: TODO
        SCARA_ARM_RADIUS: TODO
        fiducial_transforms: TODO
        read_to_pickup: TODO
        read_cv_data: TODO
        prev_transform: TODO
        prev_fid_id: 
        same_pos_counter: TODO
        colour: TODO
        colour_map: TODO
    Methods: 
        run(): TODO
        rotation_matrix(): TODO
        update_tf_msg(): TODO
        is_moving(): TODO
        find_block_transform(): TODO 


    """
    def __init__(self):
        """
        Constructs all the necessary attributes for ComputeIk object
        """    
        rospy.init_node("scara_cv", anonymous=True)
        self.block_transform_pub = rospy.Publisher("block_transform", FiducialTransform, queue_size=1)
        self.find_colour_pub = rospy.Publisher("id", Int32, queue_size=1)
        self.scara_home_sub = rospy.Subscriber("/scara_home", Bool, self.ready_callback, queue_size=1)
        self.fid_transform_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback, queue_size=1)
        self.colour_sub = rospy.Subscriber("/block_colour", String, self.colour_callback, queue_size=1)
        self.listner = tf.TransformListener()
        self.tf_msg = FiducialTransform()
        self.rate = rospy.Rate(1)

        self.Tbase_fiducial = "42"
        self.SCARA_ARM_RADIUS = 220e-3 
        self.fiducial_transforms = None
        self.ready_to_pickup = False
        self.read_cv_data = True

        self.prev_transform = None
        self.prev_fid_id = None
        self.same_pos_counter = 0
        self.colour = None
        self.colour_map = {"red": 0, "green": 1, "blue": 2, "yellow": 3}
        pass

    def run(self):
        """
        Runs spin() with a small sleep before hand to account for any errors
        """        
        rospy.sleep(3)
        rospy.spin()

    def rotation_matrix(self, rotation)->np.array:
        """
        TODO

        Args:
            rotation ([type]): [description]

        Returns:
            np.array: [description]
        """        
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

    def update_tf_msg(self, fid_id, transform):
        """
        TODO

        Args:
            fid_id (int): [description]
            transform (geometry_msgs/Transform.msg): [description]
        """        
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

    def is_moving(self, curr_trans):
        """
        Checks if given current transorm differs enough from the previous transform to declare the block moving

        Args:
            curr_trans (geometry_msgs/Transform.msg): [description]TODO

        Returns:
            True (Bool): if block is moving
            False (Bool): if block is not moving
        """        
        # Check if this is the first callback to occur
        if not self.prev_fid_id or not self.prev_transform:
            return False

        # Extract data from message type
        prev_rot_x = self.prev_transform.transform.rotation.x
        prev_rot_y = self.prev_transform.transform.rotation.y
        prev_rot_z = self.prev_transform.transform.rotation.z

        prev_trans_x = self.prev_transform.transform.translation.x
        prev_trans_y = self.prev_transform.transform.translation.y
        prev_trans_z = self.prev_transform.transform.translation.z

        curr_rot_x = curr_trans.transform.rotation.x
        curr_rot_y = curr_trans.transform.rotation.y
        curr_rot_z = curr_trans.transform.rotation.z

        curr_trans_x = curr_trans.transform.translation.x
        curr_trans_y = curr_trans.transform.translation.y
        curr_trans_z = curr_trans.transform.translation.z

        pos_error = 0.5 # TODO: Tune this error --> 0.5
        rot_error = 0.5 # TODO: Tune this error

        # Check if the rotation is the same between callbacks
        if (abs(curr_rot_x - prev_rot_x) < rot_error) and \
           (abs(curr_rot_y - prev_rot_y) < rot_error) and \
           (abs(curr_rot_z - prev_rot_z) < rot_error):
           pass
        else:
            return False
        
        # Check if the translation is the same between callbacks
        if (abs(curr_trans_x - prev_trans_x) < pos_error) and \
           (abs(curr_trans_y - prev_trans_y) < pos_error) and \
           (abs(curr_trans_z - prev_trans_z) < pos_error):
           # Check if the same position has been recorded for the 
           # last 2 callbacks
           print(self.same_pos_counter)
           if self.same_pos_counter < 2:
               self.same_pos_counter += 1
               return False
           return True
        else:
            return False

    def find_block_transform(self):
        """
        TODO

        Returns:
            (fid_id, Tbase_block): 
        """        
        for _tf in self.fiducial_transforms:
            fid_id = _tf.fiducial_id
            if fid_id != int(self.Tbase_fiducial):
                (trans, rot) = self.listner.lookupTransform(f"/fiducial_{self.Tbase_fiducial}", f"/fiducial_{fid_id}", rospy.Time(0))
                block_posx = trans[0] 
                block_posy = trans[1] 
                # The height is always constant and does not affect choice of block
                curr_dist = np.linalg.norm(np.array([block_posx, block_posy, 0]))
                if curr_dist < self.SCARA_ARM_RADIUS: # Tune this to not have internal collisions
                    euler_angles = tf.transformations.euler_from_quaternion(rot)
                    Rbase_block = self.rotation_matrix(euler_angles)
                    Pbase_block = np.array([
                                            [trans[0]],
                                            [trans[1]],
                                            [trans[2]]
                    ])
                    Tbase_block = np.block([
                                            [Rbase_block , Pbase_block],
                                            [0, 0, 0, 1]
                    ])
                    print(fid_id)
                    return (fid_id, Tbase_block)
        return (None, None)

    def colour_callback(self, data):
        """
        Function that is called when data is published to /block_transform

        Args:
            data (String): [description]TODO
        """        
        self.colour = data.data

    # This callback is given data of type Tfiduicial_camera
    def fiducial_callback(self, data):
        """
        

        Args:
            data (fiducial_msgs/FiducialTransformArray): [description]
        """        
        # Remove base fiducial from data 
        # as this is not a valid block
        for (i, _tf) in enumerate(data.transforms):
            if _tf.fiducial_id == self.Tbase_fiducial:
                del data.transforms[i]
        if self.read_cv_data and len(data.transforms) > 1:
            # Now, check a visible fiducial
            self.fiducial_transforms = data.transforms
            current_id = data.transforms[0].fiducial_id
            current_transform = data.transforms[0]

            # Check if prev fiducial id is still in frame
            if current_id != self.prev_fid_id:
                self.ready_to_pickup = False
                self.prev_fid_id = current_id
                self.prev_transform = current_transform
                return
            
            self.ready_to_pickup = self.is_moving(current_transform)
            self.prev_fid_id = data.transforms[0].fiducial_id
            self.prev_fid_id = data.transforms[0]
        else:
            # Reset the values for next callback
            self.prev_transform = None
            self.prev_fid_id = None
            self.same_pos_counter = 0
            self.fiducial_transforms = data.transforms
        self.rate.sleep()

    def ready_callback(self, data):
        """
        

        Args:
            data (Bool): [description]
        """        
        while data.data:
            try:
                if self.ready_to_pickup:
                    self.read_cv_data = False
                    if self.fiducial_transforms:
                        (fid_id, Tbase_block) = self.find_block_transform()
                        if fid_id is None or Tbase_block is None:
                            self.read_cv_data = True
                            print("All blocks found were out of reach")
                            continue
                        print("obtaining colour of block id {fid_id}")
                        id_msg = Int32()
                        id_msg.data = int(fid_id)
                        self.find_colour_pub.publish(id_msg)
                        while (not self.colour):
                            # Wait till the colour node returns a colour
                            pass
                        self.update_tf_msg(self.colour_map[self.colour], Tbase_block)
                        rospy.loginfo(self.tf_msg)
                        self.block_transform_pub.publish(self.tf_msg)
                        self.tf_msg = FiducialTransform()
                    self.read_cv_data = True
                    break
                else:
                    #print("Computer vision is still waiting for robot to move to home position.")
                    pass
            except Exception:
                print(Exception)
                pass
            pass
        pass


if __name__ == "__main__":
    try:
        rv = RobotVision()
        rv.run()
    except rospy.ROSInterruptException:
        print("An error occured running the Robot vision node.")

