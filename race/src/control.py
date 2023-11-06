#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy

# PID Control Params 12, 50
kp = 65 #TODO
kd = 180 #TODO
ki = 0.0 #TODO
servo_offset = 13.5	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 20.0	#TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	global angle_deg

	# print(prev_error)

	error = data.pid_error

	if math.isnan(error):
		# error = prev_error
		command = AckermannDrive()
		command.speed = vel_input
		command.steering_angle = 0
		command_pub.publish(command)
		return

	# print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error

	# 2. Apply the PID equation on error to compute steering
	p_term = kp * error
	d_term = kd * (error - prev_error)

	pid = p_term + d_term

	prev_error = error

	# i_term = ki * (i_term + error)
	# angle = p_term + i_term + d_term

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	angle = angle + pid
	# angle_deg = math.degrees(angle)
	angle_deg = angle + servo_offset

	if angle_deg < -100:
		angle = -100 - servo_offset
	elif angle_deg > 100:
		angle = 100 - servo_offset
	else:
		pass
	angle_deg = angle + servo_offset
	command.steering_angle = angle_deg
		# command.steering_angle = 90
		# command.steering_angle_velocity = 10

	# TODO: Make sure the velocity is within bounds [0,100]

	command.speed = vel_input * (2-abs(error))
	# command.speed = 0

	# command.steering_angle = servo_offset

	# Move the car autonomously
	command_pub.publish(command)

def joy_command_callback(data):
	global angle
	# set prev error to 0 when offboard is started
	ctl_teleop_button = 6
	ctl_offboard_button = 7
	if data.buttons[ctl_offboard_button]:
		angle = 0
		print("angle set to 0")

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	prev_error = 0
	angle = 0
	angle_deg = 0
	# kp = input("Enter Kp Value: ")
	# kd = input("Enter Kd Value: ")
	# ki = input("Enter Ki Value: ")
	# vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.Subscriber('/joy', Joy, joy_command_callback)

	rospy.spin()
