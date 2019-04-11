from ArmCommandTranslator import pwm, gotoAngle
from ArmPosFeedback import feedback_channels
from ArmConstants import *
import time

CALIB_TOLERANCE = 0.02

class ArmCalib():
        
    def __init__(self,channels):
        
        self.angle_bounds = [[0,0] for i in range(MOTOR_NUM)]
        self.current_motor = MOTOR_0
        self.channels = channels
            
    def calibration_routine(self):
        
        Calib_Phase_MIN = 0
        Calib_Phase_MAX = 1
        Calib_Phases = {Calib_Phase_MIN: 'MIN',Calib_Phase_MAX: 'MAX'}
        """ This function runs the position feedback
            calibration routine on all motors sequentially"""
        for item in range(MOTOR_NUM):
            for phase in Calib_Phases.keys():
                print("Please hold the motor # {} steady at the {} position" \
                      "for 2 seconds".format(item,Calib_Phases[phase]))
                key_interrupt = input("Please press any key when in position\n")
                time_start = time.time()
                previous_raw_feedback = 0
                while(1):
                    raw_feedback = feedback_channels[item].value
                    print(previous_raw_feedback, raw_feedback)
                    if (abs(raw_feedback - previous_raw_feedback) <= (CALIB_TOLERANCE*previous_raw_feedback)):
                        time_end = time.time()
                        if ((time_end - time_start) >= 2.0):
                            self.angle_bounds[item][phase] = raw_feedback
                            break
                    else:
                        time_start = time.time()
                    previous_raw_feedback = raw_feedback


if __name__ == '__main__':
    
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_0],0,0)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_1],0,0)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_2],0,0)
    pwm.setPWM(MOTOR_LAYOUT[MOTOR_3],0,0)
    
    time.sleep(1)
    
    calib = ArmCalib(feedback_channels)
    
    calib.calibration_routine()
    
    print("The calibration values are as follows:\n")
    print(calib.angle_bounds)

    f = open("calib_record.txt","w+")
    
    f.write(calib.angle_bounds)
    
    f.close()
    