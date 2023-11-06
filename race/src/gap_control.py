#!/usr/bin/env python
import math
import rospy
from race.msg import gap_dir
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy

servo_offset = 13.5
prev_error = 0.0


command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global vel_input
	global kp
	global angle_deg

	command = AckermannDrive()
	angle_deg = data.dir

	if angle_deg < -100:
		angle_deg = -100
	elif angle_deg > 100:
		angle_deg = 100


	command.steering_angle = data.dir
	command.speed = vel_input

	command_pub.publish(command)

if __name__ == '__main__':
	kp = rospy.get_param("/kp")
	vel_input = rospy.get_param("/vel_input")
	angle_deg = 0

	rospy.init_node('gap_control', anonymous=True)
	rospy.Subscriber("error", gap_dir, control)

	rospy.spin()
