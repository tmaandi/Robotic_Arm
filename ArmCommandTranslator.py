#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import Adafruit_PWM_Servo_Driver
import time

from ArmConstants import *
from ArmPosFeedback import feedback_channels

# Read-in the learnt motor angle bounds during calibration
f_calib = open("calib_record.txt","r")

if f_calib.mode == "r":
    # Reading the calib file, line by line and splitting at the 'comma'
    # for each line to split min/max bounds for each motor
    RAW_ANGLE_BOUNDS = [x.rstrip().split(',') for x in f_calib.readlines()]
    RAW_ANGLE_BOUNDS = [[int(x) for x in y] for y in RAW_ANGLE_BOUNDS]     
    #print("RAW_ANGLE_BOUNDS: {}".format(RAW_ANGLE_BOUNDS))
else:
    RAW_ANGLE_BOUNDS = [[0,0] for i in range(MOTOR_NUM)]
    #print("RAW_ANGLE_BOUNDS: {}".format(RAW_ANGLE_BOUNDS))

f_calib.close()
    
def motorFBConv(motor_ind,raw_angle):
    """ This method transforms the raw potentiometer feedback readings into
        physical motor joint angles as per the coordinate system"""
    raw_angle_span = RAW_ANGLE_BOUNDS[motor_ind][1] - RAW_ANGLE_BOUNDS[motor_ind][0]
    physical_angle_span = MOTOR_BOUNDS['UPPER_BOUND'][motor_ind] - MOTOR_BOUNDS['LOWER_BOUND'][motor_ind]
    
    if (raw_angle_span != 0):
        fb_angle = MOTOR_BOUNDS['LOWER_BOUND'][motor_ind] + (physical_angle_span / raw_angle_span) * (raw_angle - RAW_ANGLE_BOUNDS[motor_ind][0])
    else:
        raise "Invalid motor angle bounds"
    
    return fb_angle

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
    # Servo 1 motion should ideally be limited between 45 - 135 deg due to excess
    # loading at larger angles, not enough motor torque at channel 1

    angle = limit(angle,MOTOR_BOUNDS['LOWER_BOUND'][motor],MOTOR_BOUNDS['UPPER_BOUND'][motor])
    
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
    
    # Feedback Error
    fb_raw_angle = feedback_channels[motor].value
    fb_angle = motorFBConv(motor, fb_raw_angle)
    control_error = angle - fb_angle
    
    # Proportional Gain
    K_p = [0.6,0.2,1.0,0.4] # duty cycle count per degree of error
    # Integral Gain
    K_i = [0.06,0.06,0.1,0.1] # addition to duty cycle per loop per degree error
    # Derivative Gain
    K_d = [0.25, 0.0, 0.5, 0.00]
    
    settling_timer = 0;
    error_sum = 0
    error_derivative = 0
    prev_control_error = 0
    
    while(settling_timer < 100):
        # Closed Loop Servo Pulse
        # Use integral only when close to the target
        if (abs(control_error) < 8.0):
            error_sum += control_error
        elif (abs(control_error) > 16.0):
            pass
        #error_sum = 0
        else:
            pass
               
        print("error_sum: {}".format(error_sum))
        
        feedback_control_term = (K_p[motor] * control_error) + (K_i[motor] * error_sum) + (K_d[motor] * error_derivative)
        # MOTOR_1 increments the angle with decreasing PWM
        if (motor == MOTOR_0):
            servoPulseCL = servoPulse - feedback_control_term
        else:
            servoPulseCL = servoPulse + feedback_control_term
            
        pwm.setPWM(MOTOR_LAYOUT[motor],0,int(servoPulseCL))
        
        fb_raw_angle = feedback_channels[motor].value
        fb_angle = motorFBConv(motor, fb_raw_angle)
        control_error = angle - fb_angle
        
        if(abs(control_error) < 3.0):
            settling_timer = settling_timer + 1
        else:
            settling_timer = 0
        
        # Calculating Error Derivative
        error_derivative = control_error - prev_control_error
        prev_control_error = control_error
                    
        print("Servo Pulse: {}".format(servoPulseCL))
        print("Target: {}, Feedback: {}".format(angle,fb_angle))
        print("Control_Error: {}".format(control_error))
        print("D_Term: {}".format((K_d[motor] * error_derivative)))

##############################
#### Test Code/Manual Run ####
##############################
 
if __name__ == "__main__":
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_0],0,375)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_1],0,380)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_2],0,405)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_3],0,395)
    runMotor = False
    
    while(1):
        motor_num = int(input("Please enter the motor #: (0 or 1 or 2 or 3) "))
        if (motor_num in list(MOTOR_LAYOUT.keys())):
            try:
                angle = float(input("Please enter the angle between {} and {} deg ".format(MOTOR_BOUNDS['LOWER_BOUND'][motor_num],MOTOR_BOUNDS['UPPER_BOUND'][motor_num])))
                runMotor = True
            except:
                print("This 'angle' is not a real number. Please re-enter the motor choice and angle")
        else:
            print("Incorrect motor choice")
            runMotor = False
        if (runMotor == True):
            gotoAngle(motor_num, angle)
            time.sleep(3)
            fb_raw_angle_debug = feedback_channels[motor_num].value
            fb_angle_debug = motorFBConv(motor_num, fb_raw_angle_debug)
            print("Target: {}, Feedback: {}".format(angle,fb_angle_debug))
            
