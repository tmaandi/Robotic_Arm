from ArmKinematics import effectiveDhMatrix
from ArmCommandTranslator import gotoAngle, limit

if __name__ == "__main__":
    
    # Initial Arm Position    
    gotoAngle(0, 0)
    gotoAngle(1, 90)
    gotoAngle(2, 0)
    gotoAngle(3, 0)

    while(1):
        run_prog = True
        joint_angles = [ ]
        for index in range(4):
            if (run_prog == True):
                if (index == 1):
                    try:
                        jointAngle = float(raw_input("Please enter the rotation angle [ 45.0, 135.0] deg for motor # " + str(index) + " : "))
                        jointAngle = limit(jointAngle,  45.0, 135.0)
                        joint_angles.append(jointAngle)
                    except:
                        print "This 'angle' is not a real number. Please re-enter the motor choice and angle"
                        run_prog = False                        
                        break
                else:
                    try:
                        jointAngle = float(raw_input("Please enter the rotation angle [ -90.0, 90.0] deg for motor # " + str(index) + " : "))
                        jointAngle = limit(jointAngle,  -90.0, 90.0)
                        joint_angles.append(jointAngle)
                    except:
                        print "This 'angle' is not a real number. Please re-enter the motor choice and angle"
                        run_prog = False                        
                        break                      
    
        if (run_prog == False):
            continue              
        
        motor_index = 0

        for angle in joint_angles:
            gotoAngle(motor_index, angle)
            motor_index = motor_index + 1
        
        T = effectiveDhMatrix(joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3])
        print("\nThe coordinates of the end effector's origin are:")
        print("X: " + str(T[0,3]) + "  Y: " + str(T[1,3]) + "  Z: " + str(T[2,3]) + "\n")


