#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
# from visualization_msgs.msg import Marker
import tf
import numpy as np

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
plan_np             = []
path_resolution     = []
frame_id            = 'map'
car_name            = rospy.get_param("/car_name")
trajectory_name     = rospy.get_param("/trajectory_name")

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size=1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size=1)
map_pub         = rospy.Publisher('/map_poly', PolygonStamped, queue_size=1)
steering_pub = rospy.Publisher('/steering', PoseStamped, queue_size=1)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon
global plan_np

wp_seq          = 0
control_polygon = PolygonStamped()

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN = 0.325

def construct_path():
    global plan_np

    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('/home/nvidia/depend_ws/src/F1tenth_car_workspace/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
        dx = plan[index][0] - plan[index - 1][0]
        dy = plan[index][1] - plan[index - 1][1]
        path_resolution.append(math.sqrt(dx * dx + dy * dy))

    plan_np = np.array(plan)
    # print("Path shape:", plan_np.shape)
    



def purepursuit_control_node(data):
    start_time = rospy.get_time()
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon
    global plan_np

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.

    # Calculate the Euclidean distances from the car's current position to each point in the plan
    # ! changed this line
    # distances = [math.sqrt((odom_x - point[0]) ** 2 + (odom_y - point[1]) ** 2) for point in plan]
    # ! to this
    # 4 elements to match path np.shape
    distances = np.linalg.norm(plan_np - [odom_x, odom_y], axis=1)

    # Find the index of the closest point (base projection)
    # ! changed this line
    # base_projection_index = distances.index(min(distances))
    # ! to this
    base_projection_index = np.argmin(distances)

    # Obtain the x and y coordinates of the base projection
    pose_x = plan[base_projection_index][0]
    pose_y = plan[base_projection_index][1]

    #] Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]

    # TODO 2: You need to tune the value of the lookahead_distance
    lookahead_distance = 1.5 # Fine-tune this value according to your requirements
    # lookahead_distance = 1.9 # Fine-tune this value according to your requirements

    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    dist = 0
    tripped = False

    target_x = plan[base_projection_index][0]
    target_y = plan[base_projection_index][1]

    # for i in range(base_projection_index, len(plan)):
    #     dist = distances[i]
    #     if dist > lookahead_distance:
    #         tripped = True
    #         target_x = plan[i][0]
    #         target_y = plan[i][1]
    #         break

    # if not tripped:
    #     for i in range(0, len(plan)):
    #         dist = distances[i]
    #         if dist > lookahead_distance:
    #             tripped = True
    #             target_x = plan[i][0]
    #             target_y = plan[i][1]
    #             break

    mask = distances[base_projection_index:] > lookahead_distance

    if mask.any():
        target_index = base_projection_index + np.argmax(mask)
        target_x, target_y = plan[target_index][:2]
    else:
        target_index = 3
        target_x, target_y = plan[3][:2]

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.

    # Calculate the relative coordinates of the target point with respect to the car's current position

    dx = (target_x - odom_x)
    dy = (target_y - odom_y)

    rotate_x = dx*math.cos(heading) + dy*math.sin(heading)
    rotate_y = dy*math.cos(heading) - dx*math.sin(heading)

    # Calculate the distance to the target point
    distance_to_target = distances[target_index]

    # Calculate the steering angle using the pure pursuit formula
    # alpha = (math.asin((rotate_y)/distance_to_target))

    steering_angle = 4 * math.degrees(math.atan((2.0 * WHEELBASE_LEN * ((rotate_y)/distance_to_target))/(distance_to_target)))



    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    steering_angle = max(-STEERING_RANGE, min(STEERING_RANGE, steering_angle))
    command.steering_angle = steering_angle

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed

    command.speed = (1-(abs(steering_angle))/190) * 50
    # command.speed = (1/(steering_angle/5)**2) * 80

    command_pub.publish(command)

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

    # # These are set to zero only so that the template code builds. 
    # pose_x=0    
    # pose_y=0
    # target_x=0
    # target_y=0

    # ! comment out for speed
    map_polygon = PolygonStamped()
    map_polygon.header.frame_id = frame_id

    path = []
    for wp in plan:
        new_WP = Point32()
        new_WP.x = wp[0]
        new_WP.y = wp[1]
        path.append(new_WP)


    map_polygon.polygon.points = path
    map_polygon.header.stamp = rospy.Time.now()
    map_pub.publish(map_polygon)

    marker = PoseStamped()
    marker.header.frame_id = "car_1_base_link"
    marker.header.stamp = rospy.Time.now()

    quat = tf.transformations.quaternion_from_euler(0,0, math.radians(steering_angle))
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    steering_pub.publish(marker)


    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()

    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)


    # print("Elapsed Time (ms): ", (rospy.get_time()-start_time) * 1000)

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass