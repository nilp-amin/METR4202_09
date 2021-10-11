#!/usr/bin/python3


import numpy as np
from math import * 

l1 = 126.412e-3
l2 = 67.5e-3

print("==============ZONE 1 =============================")
p_x = -50e-3
p_y = 150e-3
#theta2: Second Link Rotation
costheta2 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
#positive
theta2_1 = atan2(costheta2, sqrt(1 - costheta2**2 ))
#negative
theta2_2 = atan2(costheta2, -sqrt(1 - costheta2**2 ))
# theta1: First Link Rotation
theta1_1 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_1), l2*sin(theta2_1))
theta1_2 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_2), l2*sin(theta2_2))
thetaarray = np.array([theta1_1, theta2_1, theta1_2, theta2_2])
thetaarray = thetaarray * 180/pi
thetaarray = np.where(thetaarray < -180, thetaarray+360, thetaarray)
thetaarray = np.where(thetaarray > 180, thetaarray-360, thetaarray)
print(thetaarray)

print("==============ZONE 2 =============================")
p_x = -150e-3
p_y = 50e-3
#theta2: Second Link Rotation
costheta2 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
#positive
theta2_1 = atan2(costheta2, sqrt(1 - costheta2**2 ))
#negative
theta2_2 = atan2(costheta2, -sqrt(1 - costheta2**2 ))
# theta1: First Link Rotation
theta1_1 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_1), l2*sin(theta2_1))
theta1_2 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_2), l2*sin(theta2_2))
thetaarray = np.array([theta1_1, theta2_1, theta1_2, theta2_2])
thetaarray = thetaarray * 180/pi
thetaarray = np.where(thetaarray < -180, thetaarray+360, thetaarray)
thetaarray = np.where(thetaarray > 180, thetaarray-360, thetaarray)
print(thetaarray)

print("==============ZONE 3 =============================")
p_x = -150e-3
p_y = -50e-3
#theta2: Second Link Rotation
costheta2 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
#positive
theta2_1 = atan2(costheta2, sqrt(1 - costheta2**2 ))
#negative
theta2_2 = atan2(costheta2, -sqrt(1 - costheta2**2 ))
# theta1: First Link Rotation
theta1_1 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_1), l2*sin(theta2_1))
theta1_2 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_2), l2*sin(theta2_2))
thetaarray = np.array([theta1_1, theta2_1, theta1_2, theta2_2])
thetaarray = thetaarray * 180/pi
thetaarray = np.where(thetaarray < -180, thetaarray+360, thetaarray)
thetaarray = np.where(thetaarray > 180, thetaarray-360, thetaarray)
print(thetaarray)

print("==============ZONE 4 =============================")
p_x = -50e-3
p_y = -150e-3
#theta2: Second Link Rotation
costheta2 = (p_x**2 + p_y**2 - l1**2 - l2**2) / (2*l1*l2)
#positive
theta2_1 = atan2(costheta2, sqrt(1 - costheta2**2 ))
#negative
theta2_2 = atan2(costheta2, -sqrt(1 - costheta2**2 ))
# theta1: First Link Rotation
theta1_1 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_1), l2*sin(theta2_1))
theta1_2 = atan2(p_x,p_y) - atan2(l1 + l2*cos(theta2_2), l2*sin(theta2_2))
thetaarray = np.array([theta1_1, theta2_1, theta1_2, theta2_2])
thetaarray = thetaarray * 180/pi
thetaarray = np.where(thetaarray < -180, thetaarray+360, thetaarray)
thetaarray = np.where(thetaarray > 180, thetaarray-360, thetaarray)
print(thetaarray)

