#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy

servo_offset = 13.5	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
# Velocity set in rosparams

# Publisher for moving the car.
command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global vel
	global kp
	global kd
	global angle
	global angle_deg
	global integral

	error = data.pid_error

	if math.isnan(error):
		command = AckermannDrive()
		command.speed = vel
		command.steering_angle = 0
		command_pub.publish(command)
		return

	## Your PID code goes here

	# 1. Scale the error

	# 2. Apply the PID equation on error to compute steering
	p_term = kp * error
	integral += error
	i_term = ki * integral
	d_term = kd * (error - prev_error)

	pid = p_term + d_term

	prev_error = error

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
	angle = pid
	angle_deg = angle

	if angle_deg < -100:
		angle_deg = -100
	elif angle_deg > 100:
		angle_deg = 100
	else:
		pass

	command.steering_angle = angle_deg

	vel = vel_input - ( (10 * abs(error)) - 5 * (error-prev_error) )

	if vel < 25:
		vel = 25

	if vel > 55:
		vel = 55
	command.speed = vel

	command_pub.publish(command)

if __name__ == '__main__':

	kp = rospy.get_param("/kp")
	kd = rospy.get_param("/kd")
	ki = rospy.get_param("/ki")
	vel_input = rospy.get_param("/vel_input")
	vel = vel_input

	angle_deg = 0

	integral = 0

	rospy.init_node('pid_controller', anonymous=True)

	rospy.Subscriber("error", pid_input, control)

	rospy.spin()
