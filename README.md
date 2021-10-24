# METR4202-09
METR4202 Team 09 ROS Code (https://github.com/Nilp-amin/METR4202_09)

This file contains the code library for a scara robot. This scara robot uses four Dynamixel AX-12A for the joints, with one of them used for the prismatic joint, and one SG90 servo for the gripping function at the end effector.

This code library contains all the required files: python scripts in each of the robot function packages, the launch files, ximea camera sub modules, fiducial sub modules, dynamixel interface sub modules, and the gazebo simulation files such as urdf files. The python scripts will have doc strings and comments on what each function does and what certain blocks of code do.

# Robot Vision Package

**Package Name: robot_vision**

Scripts:
- robot_vision.py -> Using ximea package to detect fiducials in sight
  - **Node name**: scara_cv
  - **Subscribers**: 
    - /fiducial_transforms (fiducial_msgs/FiducialTransform)
    - /scara_home (std_msgs/Bool)
  - **Publishers**: /block_transform (fiducial_msgs/FiducialTransform)
- sim_robot_vision.py -> Using ximea package to detect fiducials in simulation
- colour_picker.py -> Detects the colour in rgb/hsv values (referenced code)
- robot_colour_detect.py -> Also detects the colour (referenced code)

Launch:
- robot_planning.launch -> Launches "robot_vision.py" with the **Node name**: scara_cv.
- sim_robot_planning.launch -> Launches "sim_robot_vision.py" with the **Node name**: scara_sim_cv.

Referenced Code:
- ximea package: https://github.com/wavelab/ximea_ros_cam.git
- robot_colour_detect.py, colour_picker.py: Miguel Valencia (https://github.com/uqmvale6/metr4202_color)

Submodules:
- ximea_ros_cam
- fiducial

# Robot Kinematics Package

**Package Name: robot_kinematics**

Scripts:
- placement_angle_finder.py -> Script to find the angles the joints require for the robot to go to the specified zone.
- robot_kinematics.py -> This script does all the inverse kinematics for the robot. 
  - **Node name**: scara_kinematics  
  - **Subscribers**: /block_transform (fiducial_msgs/FiducialTransform)  
  - **Publishers**: /scara_angles (std_msgs/Float32MultiArray)
- sim_robot_kinematics.py -> The inverse kinematics script for the simulation.

Launch:
- sim_robot_kinematics.launch -> Launches "sim_robot_kinematics.py" with the **Node name**: sim_scara_ik.

# Robot Trajectory Planning Package

**Package Name: robot_planning**

Scripts:
- robot_planning.py -> Script does the robot trajectory planning, publishing the inverse kinematics values and telling the robot which joint moves first and such.
  - **Node name**: scara_trajectory 
  - **Subscribers**: 
    - /scara_angles (std_msgs/Float32MultiArray) 
    - /joint_states (sensor_msgs/JointState)
  - **Publishers**: 
    - /desired_joint_states (sensor_msgs/JointState)
    - /scara_home (std_smgs/Bool)
- sim_robot_planning.py -> Robot trajectory planning for the simulation

Launch:
- robot_planning.launch -> Launches "robot_planning.py" with the **Node name**: scara_trajectory.
- sim_robot_planning.launch -> Launches "sim_robot_planning.py" with the **Node name**: scara_trajectory.

Referenced Code:
- End effector movement: https://abyz.me.uk/rpi/pigpio/python.html

Submodules:
- dynamixel_interface

# Robot Description Package

**Package Name: scara_simple_description**

Launch:
- controller.launch -> launches controller.yaml
- display.launch -> launches the urdf.rviz
- gazebo.launch -> launches the gazebo

Meshes directory contains all the stl files for the gazebo simuation.

URDF direction contains all the required urdf files (.xacro, .gazebo, .trans).

# Master Package

**Package Name: master_launch**

Launch:
- scara.launch -> The master launch file, launching all the launch files in the packages and runs all the python scripts.

# Installation Tutorial

First make sure to be in a source file in the catkin workspace then run the following lines:

```git clone --recursive git@github.com:Nilp-amin/METR4202_09.git```

Access the controller_config.yaml file so get into the config file in the dynamixel_interface director:

```
cd dynamixel_interface
cd config
vi controller_config.yaml
```

Press I to insert into the file and add the following code after the `torque_limit: 1.0`:

```
      - id: 2
        joint_name: joint_2
        zero_pos: 512
        min_pos: 0
        max_pos: 1023

      - id: 3
        joint_name: joint_3
        zero_pos: 512
        min_pos: 0
        max_pos: 1023

      - id: 4
        joint_name: joint_4
        zero_pos: 512
        min_pos: 0
        max_pos: 1023
```

Now, access the example_cam.launch in the ximea_ros_cam submodule directory:

First make sure to be in the METR4202_09 directory.

```
cd ximea_ros_cam
cd launch
vim example_cam.launch
```
Now, add the following code after the node line:

```
        <param name="serial_no"       type="string" value="12345678" />
```
where 12345678 represents the serial no. off the camera.

Now, go back out to the METR4202_09 directory:

```
cd robot_vision
cd scripts
python colour_picker.py
```
When a gui pops up, click on a colour button then click on the same coloured block on the camera vision side of the gui then click save.

Repeat this process for all four colours.

Now to build the workspace, make sure to be in the catkin workspace

```
catkin build
source devel/setup.bash
```

# Simulation Tutorial

To launch the URDF files for a simulation, make sure to be in the catkin workspace and do the following:

```
roslaunch scara_simple_description gazebo.launch
```

Open a new terminal and run:

```
roslaunch scara_simple_description controller.launch
```

# Launch Tutorial

To launch the files, first make sure to be in the catkin workspace,

```
roslaunch master_launch scara_home.launch
```
Open a new terminal and run:
```
rostopic pub --once /scara_home std_msgs/Bool "data: true"
```

# Authors

Nilp Amin - 45811645

Ethan McDonnell - 45825530

Harry Meyers - 45317512

Justin Kim - 45890220

Lachlan Best - 45823424
