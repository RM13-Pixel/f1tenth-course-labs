#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import math


angle_range = 240	
forward_projection = 2.5	
desired_distance = 0.75 
vel = 15 		
error = 0.0		
car_length = 0.50 

pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):

	angle_rad = math.radians(angle-90)
	ind = int((angle_rad - data.angle_min) / data.angle_increment)
	
	dist = data.ranges[ind]

	return dist



def callback(data):
	global forward_projection

	theta = 75
	a = getRange(data,theta) 
	b = getRange(data, 0)
	swing = math.radians(theta)


	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	AB = (b*math.cos(alpha))


	CD = AB + forward_projection*math.sin(alpha)
	error = desired_distance-CD

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel	
	pub.publish(msg)



if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("/car_1/scan",LaserScan,callback)
	rospy.spin()
