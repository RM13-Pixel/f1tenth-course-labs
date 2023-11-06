#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import math

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.7 # distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	angle_rad = math.radians(angle-90)
	ind = int((angle_rad - data.angle_min) / data.angle_increment)
	
	dist = data.ranges[ind]

	if math.isnan(dist):
		if not math.isnan(data.ranges[ind - 1]):
			if not math.isnan(data.ranges[ind + 1]):
				dist = (data.ranges[ind + 1] + data.ranges[ind - 1])/2
			else:
				dist = data.ranges[ind - 1]
		elif not math.isnan(data.ranges[ind + 1]):
			dist = data.ranges[ind + 1]
		else:
			pass # remains nan

	# print("Range: ", range)

	return dist



def callback(data):
	global forward_projection

	theta = 75 # you need to try different values for theta
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data, 0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	swing = math.radians(theta)

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	# TODO: implement
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	AB = (b*math.cos(alpha))
	# print("AB", AB)
	
	# error = desired_distance-AB
	# if math.isnan(error):
	# 	print("Angle: ", a)
	# 	print("Right: ", b)


    # CHANGE forward_projection variable above based on speed of car
	CD = AB + forward_projection*math.sin(alpha)
	error = desired_distance-CD

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)
	# print("Message: ", msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_1/scan",LaserScan,callback)
	rospy.spin()
