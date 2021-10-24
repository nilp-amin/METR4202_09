# METR4202-09
METR4202 Team 09 ROS Code

This file contains the code library for a scara robot. This code will allow the scara robot to execute and perform its task. This scara robot uses four Dynamixel AX-12A for the joints, with one of them used for the prismatic joint, and one SG90 servo for the gripping function at the end effector.

This code library contains all the required files: python scripts in each of the robot function packages, the launch files, ximea camera sub modules, fiducial sub modules, dynamixel interface sub modules, and the gazebo simulation files such as urdf files. The python scripts will have doc strings and comments on what each function does and what certain blocks of code do.

# Packages

**robot_kinematics package**

Scripts:
- placement_angle_finder.py -> Script to find the angles the joints require for the robot to go to the specified zone.
- robot_kinematics.py -> This script does all the inverse kinematics for the robot. 
  - **Node name**: scara_kinematics  
  - **Subscribers**: /block_transform  
  - **Publishers**: /scara_angles
- sim_robot_kinematics.py -> The inverse kinematics script for the simulation.

Launch:
- sim_robot_kinematics.launch -> Launches "sim_robot_kinematics.py" with the **Node name**: sim_scara_ik.

