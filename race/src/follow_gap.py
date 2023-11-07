#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import numpy as np

command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)
laser_pub = rospy.Publisher('/extended_laser', LaserScan, queue_size = 1)

#! only for testing lidar values 
def getRange(data, angle):

    angle_rad = math.radians(angle-90)
    ind = int((angle_rad - data.angle_min) / data.angle_increment)

    print(ind)

    dist = data.ranges[ind]

    return dist

def extendDisparities(rangeList, angle_increment):
    global threshold
    halfCarWidth = 0.35

    ranges = np.array(rangeList)

    disparities = np.where(np.abs(np.diff(ranges)) > threshold)[0]

    ranges = rangeList
    new_ranges = ranges

    print("++++++++++++++++++++++++++++++++++++++++++++")
    print(disparities)
    for i in disparities:
        if ranges[i] < ranges[i+1] and ranges[i] != 0:
                #! horizontal distance between two ranges = range * sin(angle difference) 
                dist = ranges[i] * math.sin(angle_increment)
                numSamples = int(math.ceil(halfCarWidth / dist))

                #* 3. Starting at the more distant of the two points and continuing in the same direction, overwrite the number of samples in the array
                #* with the closer distance. Do not overwrite any points that are already closer!
                for j in range(numSamples):
                    if i+1+j < len(ranges):
                        new_ranges[i+1+j] = ranges[i]
        elif ranges[i+1] < ranges[i] and ranges[i+1] != 0:
                #! horizontal distance between two ranges = range * sin(angle difference) 
                dist = ranges[i+1] * math.sin(angle_increment)
                numSamples = int(math.ceil(halfCarWidth / dist))
                # print("Left: ",numSamples)

                #* 3 (again). Starting at the more distant of the two points and continuing in the same direction, overwrite the number of samples in the array
                #* with the closer distance. Do not overwrite any points that are already closer!
                for j in range(numSamples):
                    if i-j > -1:
                        new_ranges[i-j] = ranges[i+1]
    return new_ranges

def getMaxIndNaive(rangeList):
    maxRange = 0
    maxInd = -1

    # #TODO naive - maybe use idxmax
    for i in range(len(rangeList)):
        if rangeList[i] > maxRange:
            maxRange = rangeList[i]
            maxInd = i

    return maxInd

def closestStraight(rangeList):
    neg90 = 127 # RIGHT!
    pos90 = 639 # LEFT!
    total = 725

    gaps2 = []
    max_dist = 0
    for i in range(neg90, pos90):
        if (rangeList[i] > max_dist):
            max_dist = rangeList[i]
            maxInd = i
            if len(gaps2)<2:
                gaps2.append((max_dist, maxInd))
            else:
                if abs(gaps2[0][0] > max_dist):
                    gaps2[0] = (max_dist, maxInd)
                else:
                    gaps2[1] = (max_dist, maxInd)

    if gaps2[0][0] > gaps2[1][0]:
        maxInd = gaps2[1][1]
    else:
        maxInd = gaps2[0][1]

    return maxInd

def findLargestGap(rangeList):
    maxSize = 0
    maxStart = None

    currSize = 0
    currStart = None

    for i in range(len(rangeList)):
        if rangeList[i] == 0:
            if currSize > maxSize:
                maxSize = currSize
                maxStart = currStart
            currSize = 0
            currStart = None
        else:
            if currStart == None:
                currStart = i
            currSize += 1

    if currSize > maxSize:
                maxSize = currSize
                maxStart = currStart

    if maxStart != None:
        middle = maxStart + (maxSize-1) //  2
        return middle
    else:
        maxRange = -1
        maxInd = -1
        for i in range(len(rangeList)):
            if rangeList[i] > maxRange:
                maxRange = rangeList[i]
                maxInd = i
        return maxInd

def callback(data):
    global threshold
    global velocity

    #* 1.Take the raw array of Lidar samples. Find disparities in the Lidar readings. A disparity is two subsequent points in the
    #* array of distance values that differ by some amount larger than some predefined threshold. Threshold of 0.1m is a good starting
    #* point but try different values.
    ranges = list(data.ranges)

    #TODO NaNs?
    ranges = [10 if math.isnan(x) else x for x in ranges] 
    ranges = [0 if x < 0.01 else x for x in ranges] 

    neg90 = 127 # RIGHT!
    pos90 = 639 # LEFT!
    total = 725
    # print(ranges)
    # ! remove values beyond 90 deg
    ranges[:neg90-1] = [0] * (neg90 - 1)
    ranges[pos90+1:] = [0] * (total - pos90 - 1)

    #* 2 and 3 in function 
    ranges = extendDisparities(ranges, data.angle_increment)

    laser = LaserScan()
    laser = data
    laser.ranges = tuple(ranges)
    laser_pub.publish(laser)

    #* 4. Search through these filtered distances corresponding to angles between -90 and +90 degrees (to make sure we don't identify
    #* a path behind the car).
    
    
    #* 5. Out of Order -- after 6
    #* 6. Here you can be creative and if there are multiple candidate gaps then you can choose the fatherst/deepest gap or the center of
    #* the widest gap. Try what works best

    maxInd = getMaxIndNaive(ranges)
    # maxInd = closestStraight(ranges)
    # maxInd = findLargestGap(ranges)
    maxRange = ranges[maxInd]
    # print(maxInd)
    # print(maxRange)

    #* 5. Once you find the sample with the farthest distance in this range, calculate the corresponding angle--that's the direction you
    #* want to target.
    angle = data.angle_min + (maxInd * data.angle_increment)

    #* 7. Actuate the car to move towards this goal point by publishing an `AckermannDrive message to the topic as you did for Wall
    #* Following - the same ranges for steering [-100,100] and velocity [0,100] apply.
    command = AckermannDrive()

    angle_deg = math.degrees(angle)
    # print(angle_deg)

    if angle_deg < -100:
        angle_deg = -100
    elif angle_deg > 100:
        angle_deg = 100


    command.steering_angle = angle_deg

    #* choose speed based on distance
    # speed = velocity + (1 * maxRange)
    speed = velocity
    command.speed = speed

    command_pub.publish(command)
    #* 8. We will move the obstacles around during the demo and the car should be able to avoid them.



if __name__ == '__main__':
	print("Follow The Gap started")
    
	threshold = rospy.get_param("/threshold")
	velocity = rospy.get_param("/velocity")


	rospy.init_node('follow_gap',anonymous = True)

	rospy.Subscriber("/car_1/scan", LaserScan, callback)

	rospy.spin()