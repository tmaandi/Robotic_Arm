# Define motor indices
MOTOR_0 = 0
MOTOR_1 = 1
MOTOR_2 = 2
MOTOR_3 = 3

# Define motor channels
MOTOR_LAYOUT = {MOTOR_0: 0, MOTOR_1: 1, MOTOR_2: 2, MOTOR_3: 4}

# Define motor bounds
MOTOR_BOUNDS = {'LOWER_BOUND':[-90.0,45.0,-90.0,-90.0], 'UPPER_BOUND':[90.0,135.0,90.0,90.0]}

# Inverse Kinematics
linkLength = 120.0 # mm
base0height = 5.0 # mm
base1height = 82.98 # mm
motor1height = base0height + base1height