#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import numpy as np

command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)
laser_pub = rospy.Publisher('/extended_laser', LaserScan, queue_size = 1)
gap_pub = rospy.Publisher('/gap_point', LaserScan, queue_size = 1)

#! only for testing lidar values 
def getRange(data, angle):

    angle_rad = math.radians(angle-90)
    ind = int((angle_rad - data.angle_min) / data.angle_increment)

    print(ind)

    dist = data.ranges[ind]

    return dist

def extendDisparities(rangeList, angle_increment):
    threshold = 0.1
    halfCarWidth = 0.15
    width_tolerance = .15
    ranges = np.array(rangeList)

    disparities = np.where(np.abs(np.diff(ranges)) > threshold)[0]

    ranges = rangeList
    new_ranges = ranges

    for i in disparities:
        if ranges[i] < ranges[i+1] and ranges[i] != 0:
            #! horizontal distance between two ranges = range * sin(angle difference) 
            dist = ranges[i] * math.sin(angle_increment)
            numSamples = int(math.ceil((halfCarWidth + width_tolerance) / dist))

            #* 3. Starting at the more distant of the two points and continuing in the same direction, overwrite the number of samples in the array
            #* with the closer distance. Do not overwrite any points that are already closer!
            for j in range(numSamples):
                if i+1+j < len(ranges):
                    if new_ranges[i+1+j] < ranges[i]:
                        break
                    new_ranges[i+1+j] = ranges[i]

        elif ranges[i+1] < ranges[i] and ranges[i+1] != 0:
            #! horizontal distance between two ranges = range * sin(angle difference) 
            dist = ranges[i+1] * math.sin(angle_increment)
            numSamples = int(math.ceil((halfCarWidth + width_tolerance) / dist))
            # print("Left: ",numSamples)

            #* 3 (again). Starting at the more distant of the two points and continuing in the same direction, overwrite the number of samples in the array
            #* with the closer distance. Do not overwrite any points that are already closer!
            for j in range(numSamples):
                if i-j > -1:
                    if new_ranges[i-j] < ranges[i+1]:
                        break
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

    # print(maxInd)
    return maxInd

def getMaxIndCenter(rangeList):
    maxRange = 0
    maxInd = -1

    threshold = 2

    # #TODO naive - maybe use idxmax
    for i in range(len(rangeList)):
        if rangeList[i] > maxRange:
            maxRange = rangeList[i]
            maxInd = i

    leftInd = maxInd
    while rangeList[leftInd] > threshold and leftInd > 0:
        leftInd -= 1

    rightInd = maxInd
    while rangeList[rightInd] > threshold and rightInd < len(rangeList):
        rightInd += 1

    # print(maxInd)
    return (rightInd + leftInd)/2

def findLargestGap(rangeList):
    rangeList = np.array(rangeList)
    shoulderThreshold = 2
    shoulderInds = np.where(rangeList < shoulderThreshold)
    
    maxIndStart = np.argmax(np.diff(shoulderInds))

    idx = (shoulderInds[0].tolist()[maxIndStart] + shoulderInds[0].tolist()[maxIndStart + 1])/2

    if rangeList[idx] < 1:
        idx = 127 if rangeList[127]>rangeList[639] else 639

    return idx

def find_deepest_avg_gap(rangeList):
    ranges = np.array(rangeList)
    shoulder_threshold = 2
    shoulder_inds = np.where(ranges < shoulder_threshold)
    # print(shoulder_inds)
    # for i in range(len(shoulder_inds)-1):
    #     dists = np.array
    #     dist = shoulder_inds[i]
    #     while dist<shoulder_inds[i+1]:
    #         dists.append(dist)

    # shoulders = np.where(ranges > shoulder_threshold)[0]
    max_avg = 0
    pair = (0,0)
    shoulder_inds = shoulder_inds[0].tolist()
    for i in range(len(shoulder_inds) - 1):
        # print("range")
        start = shoulder_inds[i]
        end = shoulder_inds[i+1]
        between_elements = ranges[start+1:end]

        if(len(between_elements) > 0):
            # print("avg")
            avg = np.mean(between_elements) * (end-start)

            if avg > max_avg + 140:

                # print(avg-max_avg)
                max_avg = avg
                pair = (start, end)

    if pair == (0,0):
        maxInd = getMaxIndNaive(rangeList)
        return (maxInd-1, maxInd+1)
        
    return pair

def scale_vel(maxRange):
    vel = 0
    if maxRange > 2.8:
        vel = 30
    elif 1.3 < maxRange < 2.8:
        vel = 20
    elif maxRange <1.3:
        vel = 15

    return vel

def scale_vel_avg_dist(velocity, ranges):
    ranges = np.array(ranges)
    avg_dist = np.mean(ranges)
    print("mean: ", avg_dist)

    velocity -= 3/avg_dist

    print(velocity)
    velocity = 35 if velocity>35 else velocity
    velocity = 25 if velocity<25 else velocity
    
    return velocity

def bubble(rangeList):
    minInd = np.argmin(rangeList)
    newRanges = rangeList

    for i in range(100):
        if minInd - i < 0 or minInd + i >= len(rangeList):
            break
        newRanges[minInd - i] = 0
        newRanges[minInd + i] = 0

    return newRanges


def callback(data):
    # print("_________________________________")

    #* 1.Take the raw array of Lidar samples. Find disparities in the Lidar readings. A disparity is two subsequent points in the
    #* array of distance values that differ by some amount larger than some predefined threshold. Threshold of 0.1m is a good starting
    #* point but try different values.
    ranges = list(data.ranges)

    ranges = [5 if math.isnan(x) else x for x in ranges] 
    ranges = [0 if x < 0.05 else x for x in ranges] 
    ranges = [x * 0.96 for x in ranges]
    # ranges = [x - 0.05 for x in ranges]

    neg90 = 127 # RIGHT! 127
    pos90 = 639 # LEFT! 639
    total = 725 #725
    # print(ranges)
    # ! remove values beyond 90 deg
    ranges[:neg90-1] = [0] * (neg90 - 1)
    ranges[pos90+1:] = [0] * (total - pos90 - 1)

    #* 2 and 3 in function 
    ranges = extendDisparities(ranges, data.angle_increment)

    # ranges = bubble(ranges)

    laser = LaserScan()
    laser = data
    laser.ranges = tuple(ranges)
    laser_pub.publish(laser)

    #* 4. Search through these filtered distances corresponding to angles between -90 and +90 degrees (to make sure we don't identify
    #* a path behind the car).
    
    
    #* 5. Out of Order -- after 6
    #* 6. Here you can be creative and if there are multiple candidate gaps then you can choose the fatherst/deepest gap or the center of
    #* the widest gap. Try what works best

    # maxInd = findLargestGap(ranges)
    # maxInd = getMaxIndNaive(ranges)
    # maxInd = getMaxIndCenter(ranges)
    maxInd = (find_deepest_avg_gap(ranges)[0] + find_deepest_avg_gap(ranges)[1])/2

    maxRange = ranges[maxInd]

    laserList = [0] * 725
    laserList[maxInd] = 0.75
    gap = LaserScan()
    gap = data
    gap.ranges = tuple(laserList)
    gap_pub.publish(gap)

    #* 5. Once you find the sample with the farthest distance in this range, calculate the corresponding angle--that's the direction you
    #* want to target.

    angle = data.angle_min + (maxInd * data.angle_increment)

    #* 7. Actuate the car to move towards this goal point by publishing an `AckermannDrive message to the topic as you did for Wall
    #* Following - the same ranges for steering [-100,100] and velocity [0,100] apply.
    command = AckermannDrive()
    angle_deg = math.degrees(angle)

    if angle_deg < -100:
        angle_deg = -100
    elif angle_deg > 100:
        angle_deg = 100

    command.steering_angle = angle_deg

    #* choose speed based on distance
    # velocity = scale_vel(maxRange)
    velocity = scale_vel(ranges[(383+maxInd)/2])
    # velocity = scale_vel((ranges[maxInd] + ranges[383])/2)
    # velocity = scale_vel_avg_dist(20, ranges)

    command.speed = velocity

    command_pub.publish(command)
    #* 8. We will move the obstacles around during the demo and the car should be able to avoid them.



if __name__ == '__main__':
	print("Follow The Gap started")

	rospy.init_node('follow_gap',anonymous = True)

	rospy.Subscriber("/car_1/scan", LaserScan, callback)

	rospy.spin()