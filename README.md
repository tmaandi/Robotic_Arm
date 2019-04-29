# Robotic Arm Project

Overview
---

This project involved designing, 3D printing and controlling a 4 degree of freedom Robotic Arm. The Arm is powered by analog feedback servo motors from Adafruit. The Arm Kinematics are written using Denavit-Hartenberg convention. 

Software:
	* ArmCalib.py: This file houses the calibration code for the arm. It can be used to set the min/max feedback angles for each of the Arm's joints.
	* ArmCommandTranslator.py: This file houses the low level control codefor the Arm, converting the target joint angle into appropriate Servo PWM.
	* ArmKinematics.py: This file houses the Arm Kinematics (Forward and Inverse) based on the D-H convention.
	* ArmPosFeedback.py: This file processes the raw anaog feedback from the servo motors into physical arm joint angles.

Hardware:
	* Motors: Analog Feedback Servo (https://www.adafruit.com/product/1404)
	* Development Board:  Raspberry Pi 3 - Model B - ARMv8 with 1G RAM
	* Motor Driver: Adafruit 16-Channel PWM / Servo HAT for Raspberry Pi - Mini Kit
	* ADC IC: MCP3008 - 8-Channel 10-Bit ADC With SPI Interface

![] (RoboticArm_IMG.jpg)
![] (RoboticArm.png)