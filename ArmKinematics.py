import numpy as np
from math import cos, sin, pi, atan2, sqrt

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
 
###################### Inverse Kinematics ##########################
#### Solving for the first three angles to achieve the end effector origin position  #####
##############################################################

def inverseKinPos(x, y, z):
    """ This function solves for the requisite "theta" for 
         the first three motors to achieve a desired position
         in space for the end effector's origin """
        
    d1 = motor1height    # mm
    a = linkLength # mm

    # Calculating theta1 (Motor # 0)
    theta1 = atan2(y,x)
    
    if (theta1 > pi/2):
        theta1 = theta1 - pi
    elif (theta1 < -pi/2):
        theta1 = theta1 + pi
    else:
        pass

    # Calculating theta3 (Motor # 2)
    r = sqrt(x**2 + y**2)

    s = z - d1
    
    D = (r**2 + s**2 - 2*(a**2))/(2*(a**2))  # Cos(theta3) = D, using cosine rule

    theta3 = atan2(sqrt(1 - D**2),D) 
    # actually theta3  can be +/- both which leads to two different theta2 values, 
    # both sets lead to same end effector position, we'll stick to + only for now

    # Calculating theta2 (Motor # 1)

   # if the end effector origin's projection is in 2nd or 3rd quadrant
    if((np.sign(x) == -1) or (np.sign(y)*np.sign(theta1) == -1)):
        r = -r

    theta2 = atan2(s,r) - atan2(a*sin(theta3),a*(1 + cos(theta3)))
    print("theta2 = " + str(atan2(s,r)*180/pi) + " - " + str(atan2(a*sin(theta3),a*(1 + cos(theta3)))*180/pi))


    return [theta1*180/pi, theta2*180/pi, theta3*180/pi]
    






