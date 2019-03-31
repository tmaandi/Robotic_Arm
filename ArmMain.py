from ArmKinematics import effectiveDhMatrix,  inverseKinPos
from ArmCommandTranslator import gotoAngle, limit

import numpy as np
from ArmConstants import *

if __name__ == "__main__":
    
    # Initial Arm Position    
    gotoAngle(MOTOR_0, 0)
    gotoAngle(MOTOR_1, 90)
    gotoAngle(MOTOR_2, 0)
    gotoAngle(MOTOR_3, 0)
    
    motors = MOTOR_LAYOUT.keys()

    while(1):
        run_prog = True
        joint_angles = [ ]
        for index in range(0,4):
            if (run_prog == True):
                if (index == 1):
                    try:
                        jointAngle = float(raw_input("Please enter the rotation angle [ 45.0, 135.0] deg for motor # " + str(motors[index]) + " : "))
                        jointAngle = limit(jointAngle,  45.0, 135.0)
                        joint_angles.append(jointAngle)
                    except:
                        print("This 'angle' is not a real number. Please re-enter the motor choice and angle")
                        run_prog = False                        
                        break
                else:
                    try:
                        jointAngle = float(raw_input("Please enter the rotation angle [ -90.0, 90.0] deg for motor # " + str(motors[index]) + " : "))
                        jointAngle = limit(jointAngle,  -90.0, 90.0)
                        joint_angles.append(jointAngle)
                    except:
                        print("This 'angle' is not a real number. Please re-enter the motor choice and angle")
                        run_prog = False                        
                        break                      
    
        if (run_prog == False):
            continue              
        
        motor_index = 0

        for angle in joint_angles:
            gotoAngle(motors[motor_index], angle)
            motor_index = motor_index + 1
        
        T = effectiveDhMatrix(joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3])
        print("\nThe coordinates of the end effector's tip are:")
        print("X: " + str(T[0,3]) + "  Y: " + str(T[1,3]) + "  Z: " + str(T[2,3]) + "\n")

        # Tip of end effector
        p_tip = T[:,3]
        
        # For base of end effector
        a = np.array([120,0,0,0])
        p_base = p_tip - T.dot(a.transpose())
        
        x = float(p_base[0])
        y = float(p_base[1])
        z = float(p_base[2])
        
        print("\nThe coordinates of the end effector's base are:")
        print(p_base)
        print("\nThe angles calculated by inverse kinematics for this end-effector base position are: ")
        print(inverseKinPos(x, y, z))


