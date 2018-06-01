import numpy as np
from math import cos, sin, pi

def dhMatrix( linkLength, linkTwist, linkOffset, jointAngle):
    """ This function calculates the Denavit-Hartenberg transformation
         matrix to represent the effective homogenous coordinate 
         transformation for a given joint """
    a = linkLength # mm
    alpha = linkTwist * pi / 180.0 # Degree to Radian
    d = linkOffset # mm
    theta = jointAngle * pi / 180.0 # Degree to Radian
    
    A = np.array(  [[ cos(theta),  -sin(theta) * cos(alpha),   sin(theta) * sin(alpha), a * cos(theta)],
                              [ sin(theta),   cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                              [ 0               ,              sin(alpha)           ,            cos(alpha)           ,         d            ],
                              [ 0               ,                     0                    ,                    0                   ,          1           ]])
    return A

###################### Forward Kinematics #########################
###### Effective DH Homogenous Transformation Matrix for this Robotic Arm ######
##############################################################

linkLength = 120.0 # mm
base1height = 5.0 # mm
base2height = 82.98 # mm
motor1height = base1height + base2height

def effectiveDhMatrix(jointAngle0, jointAngle1, jointAngle2, jointAngle3):
    """ This function calcualtes the resultant homogenous transformation
         matrix, representing transformation from end effector frame to 
         base frame """

    A0 = dhMatrix(        0        , 90.0, motor1height, jointAngle0)
    A1 = dhMatrix(linkLength,   0.0,                   0.0, jointAngle1)
    A2 = dhMatrix(linkLength,   0.0,                   0.0, jointAngle2)
    A3 = dhMatrix(linkLength,   0.0,                   0.0, jointAngle3)

    T1 = np.dot(A0 , A1)
    T2 = np.dot(A2, A3)
    T = np.dot(T1, T2)

    return T
 





