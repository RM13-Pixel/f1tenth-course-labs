#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
from race.msg import gap_dir
from ackermann_msgs.msg import AckermannDrive
import math


vel = 15
v_thresh = .1
w_car = .15
pub = rospy.Publisher('error', gap_dir, queue_size=10)


def read_dists(data):
    dist = data.ranges
    dist = list(dist)

    for i in range(len(dist)-1):
        if abs(dist[i+1]-dist[i]) > v_thresh:

            if (dist[i] < dist[i+1]):
                closer_point = dist[i]*math.sin(data.angle_increment)
                num_samples = int((w_car)/closer_point)

                for j in range(i+1, i+1+num_samples):
                    if (j < len(dist)-1):
                        if (dist[j]*math.sin(data.angle_increment) >= closer_point):
                            dist[j] = closer_point

            elif (dist[i] > dist[i+1]):
                closer_point = dist[i+1]*math.sin(data.angle_increment)
                num_samples = int(w_car/closer_point)

                for j in range(i-1-num_samples,i-1):
                    if (j > 0):
                        if (dist[j]*math.sin(data.angle_increment) >= closer_point):
                            dist[j] = closer_point

    start_ind = int((-1.571-data.angle_min)/(data.angle_increment))
    end_ind = int((1.571-data.angle_min)/(data.angle_increment))
    max_dist = dist[0]
    maxInd = 0


# ==============================NAIVE===================================
    for i in range(start_ind, end_ind):
        if (dist[i] > max_dist):
            max_dist = dist[i]
            maxInd = i
    return data.angle_min + maxInd*data.angle_increment

# ==============================NAIVE===================================

# ==========================CLOSEST STRAIGHT============================
    # gaps2 = []

    # for i in range(start_ind, end_ind):
    #     if (dist[i] > max_dist):
    #         max_dist = dist[i]
    #         maxInd = i
    #         if len(gaps2)<2:
    #             gaps2.append((max_dist, maxInd))
    #         else:
    #             if abs(gaps2[0][0]) > abs(gaps2[1][0]):
    #                 gaps2[0][0] = (max_dist, maxInd)
    #             else:
    #                 gaps2[1][0] = (max_dist, maxInd)

# ==========================CLOSEST STRAIGHT============================


def callback(data):

    maxInd = read_dists(data)
    msg = gap_dir()
    msg.dir = maxInd
    pub.publish(msg)

    angle = data.angle_min + (maxInd*data.angle_increment)

if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('gap_finder',anonymous = True)
	rospy.Subscriber("/car_1/scan",LaserScan,callback)
	rospy.spin()
