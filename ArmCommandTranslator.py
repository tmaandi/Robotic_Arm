#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import Adafruit_PWM_Servo_Driver
import time

from ArmConstants import *
# Initialise the PWM device using the default address

pwm = Adafruit_PWM_Servo_Driver.PWM(0x40)
# Set frequency to 60 Hz
pwm.setPWMFreq(60)
# Command angle in degrees to servos

# Limit function
def limit(n, minn, maxn):
    return max(min(maxn, n), minn)

def gotoAngle(motor,angle):
    """
    This function translates the 'angle of rotation' requests to 
    the servos into PWM pulse while keeping in mind two things:
    1. The range of operation of each servo 
    2. The 'trim' values for each servo for mean and extreme positions
     
    """
    # Limiting the angle requests as per the motor range constraints
    if((motor == MOTOR_0) or (motor == MOTOR_2) or (motor == MOTOR_3)):
        if int(angle) > MOTOR_BOUNDS['UPPER_BOUND'][motor]:
            angle =  90.0
            print("Angle truncated to +90 deg limit")
        elif int(angle) < -90.0:
            angle = -90.0
            print("Angle truncated to -90 deg limit")
        else:
            pass
    elif (motor == MOTOR_1):
    # Servo 1 motion limited between 45 - 135 deg due to excess loading at 
    # larger angles, not enough motor torque at channel 1
        if int(angle) <  45.0:
            angle =  45.0    
            print("Angle truncated to +45 deg limit")       
        elif int(angle) > 135.0:
            angle = 135.0
            print("Angle truncated to +135 deg limit")
        else:
            pass
    else:
        pass

    # Translating angle into PWM command
    if (motor == MOTOR_0):
        # Different trims due to non-uniform 
        # behaviour in +/- direction for this 
        # motor
        if int(angle) >= 0.0:
            servoPulse = 375 - angle*230/90
        else:
            servoPulse = 375 - angle*260/90
    elif (motor == MOTOR_1):
        servoPulse = 160 + angle*220/90
    elif (motor == MOTOR_2):
        # Different trims due to non-uniform 
        # behaviour in +/- direction for this 
        # motor
        if int(angle) >= 0:
            servoPulse = 405 + angle*255/90
        else:
            servoPulse = 405 + angle*234/90
    elif (motor == MOTOR_3):
        # Different trims due to non-uniform 
        # behaviour in +/- direction for this 
        # motor
        if int(angle) >= 0:
            servoPulse = 395 + angle*263/90
        else:
            servoPulse = 395 + angle*235/90
    else:
        print("Invalid Motor Choice")
    servoPulse = int(servoPulse)
    pwm.setPWM(MOTOR_LAYOUT[motor],0,servoPulse)


#########################
#### Test Code/Manual Run ####
#########################
 
if __name__ == "__main__":
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_0],0,375)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_1],0,380)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_2],0,405)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_3],0,395)
    runMotor = False
    
    while(1):
        motor_num = int(raw_input("Please enter the motor #: (0 or 1 or 2 or 3) "))
        if ((motor_num == MOTOR_0) or (motor_num == MOTOR_2) or (motor_num == MOTOR_3)):
            try:
                angle = float(raw_input("Please enter the angle between -90 and 90 deg "))
                runMotor = True
            except:
                print("This 'angle' is not a real number. Please re-enter the motor choice and angle")
        elif (motor_num == MOTOR_1):
            try:
                angle = float(raw_input("Please enter the angle between 45 and 135 deg "))
                runMotor = True
            except:
                print("This 'angle' is not a real number. Please re-enter the motor chioce and angle")
        else:
            print("Incorrect motor choice")
            runMotor = False
        if (runMotor == True):
            gotoAngle(motor_num, angle)
            
