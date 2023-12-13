#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from collections import deque
from std_msgs.msg import Int64


ftg_pub = rospy.Publisher('/ftg', AckermannDrive, queue_size = 1)
command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)
laser_pub = rospy.Publisher('/extended_laser', LaserScan, queue_size = 1)
gap_pub = rospy.Publisher('/gap_point', LaserScan, queue_size = 1)

heading = 383

gaps = deque(maxlen=2)

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
    width_tolerance = .4
    ranges = np.array(rangeList)

    disparities = np.where(np.abs(np.diff(ranges)) > threshold)[0]

    ranges = rangeList
    new_ranges = ranges

    for i in disparities:
        if ranges[i] < ranges[i+1] and ranges[i] != 0:
            #! horizontal distance between two ranges = range * sin(angle difference) 
            dist = ranges[i] * math.sin(angle_increment)
            numSamples = int(math.ceil((halfCarWidth + width_tolerance) / dist))

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
    shoulder_threshold = 1.5
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
            # avg = np.mean(between_elements) * (end-start)
            avg = np.mean(between_elements)

            # if avg > max_avg + 200 and abs(383 - (start+end)/2) < abs(383 - (pair[0] + pair[1])/2):
            if avg > max_avg:

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
        vel = 50
    elif 1.3 < maxRange < 2.8:
        vel = 40
    elif maxRange <1.3:
        vel = 25

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
    start_time = rospy.get_time()
    # print("_________________________________")

    ranges = list(data.ranges)

    ranges = [5 if math.isnan(x) else x for x in ranges] 
    ranges = [0 if x < 0.05 else x for x in ranges] 
    # ranges = [x * 0.96 for x in ranges]
    # ranges = [x - 0.05 for x in ranges]

    neg90 = 127 # RIGHT! 127
    pos90 = 639 # LEFT! 639
    total = 725 #725
    # print(ranges)
    # ! remove values beyond 90 deg
    ranges[:neg90-1] = [0] * (neg90 - 1)
    ranges[pos90+1:] = [0] * (total - pos90 - 1)

    ranges = extendDisparities(ranges, data.angle_increment)

    # ranges = bubble(ranges)

    laser = LaserScan()
    laser = data
    laser.header.frame_id = "car_1_laser"
    laser.ranges = tuple(ranges)
    laser_pub.publish(laser)

    # maxInd = findLargestGap(ranges)
    # maxInd = getMaxIndNaive(ranges)
    # maxInd = getMaxIndCenter(ranges)
    maxInd = (find_deepest_avg_gap(ranges)[0] + find_deepest_avg_gap(ranges)[1])/2

    gaps.append(maxInd)
    gaps_arr = np.array(gaps)
    diffs = np.absolute(gaps_arr - heading)
    chosen = gaps_arr[np.argmin(diffs)]

    maxRange = ranges[chosen]

    laserList = [0] * 725
    laserList[chosen] = 0.75
    gap = LaserScan()
    gap = data
    gap.ranges = tuple(laserList)
    gap_pub.publish(gap)

    angle = data.angle_min + (chosen * data.angle_increment)

    command = AckermannDrive()
    

    # velocity = scale_vel(maxRange)
    # velocity = scale_vel((ranges[maxInd] + ranges[383])/2)
    # velocity = scale_vel_avg_dist(20, ranges)

    # mult = velocity/50 * 1.2

    angle_deg = math.degrees(angle) * 1.2

    mult = 1

    if angle_deg <= -100:
        angle_deg = -100
        mult = 0.75
    elif angle_deg >= 100:
        angle_deg = 100
        mult = 0.75

    command.steering_angle = angle_deg

    speed = 40

    command.speed = mult * (((1-(abs(angle_deg)/150)) * speed/2) + ((chosen/4) * speed/2))


    # command_pub.publish(command)
    ftg_pub.publish(command)
    # print("Elapsed: ", (rospy.get_time()-start_time)*1000)


def setHeading(data):
    global heading
    heading = data.data

if __name__ == '__main__':
	print("Follow The Gap started")

	rospy.init_node('follow_gap',anonymous = True)

	rospy.Subscriber("/car_1/scan", LaserScan, callback)
	rospy.Subscriber("/alpha", Int64, setHeading)

	rospy.spin()