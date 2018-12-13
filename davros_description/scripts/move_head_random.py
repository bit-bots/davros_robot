#!/usr/bin/env python3

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from geometry_msgs.msg import Vector3
import random
from sensor_msgs.msg import JointState

rospy.init_node("random_head", anonymous=False)

rate = rospy.Rate(100)
speed = 0.1
# how close the goal position should be reached before taking another one
precision = 0.1

goal = [0, 0, 0]
current_pos = [0, 0, 0]
goal_current_distance = -1


def cb(msg):
    global goal_current_distance
    goal_current_distance = 0
    i = 0
    for name in msg.name:
        if name == "RShoulderPitch":
            goal_current_distance += abs(msg.position[i] - goal[0])
            current_pos[0] = msg.position[i]
            continue
        elif name == "LShoulderPitch":
            goal_current_distance += abs(msg.position[i] - goal[1])
            current_pos[1] = msg.position[i]
            continue
        elif name == "RShoulderRoll":
            goal_current_distance += abs(msg.position[i] - goal[2])
            current_pos[2] = msg.position[i]
            continue
        else:
            rospy.logerr("Joint names do not match")
        i += 1


joint_goal_publisher = rospy.Publisher('/davros_motor_goals', Vector3, queue_size=1)
joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, cb)

msg = Vector3()

while not rospy.is_shutdown():
    # generate next random end goal
    head_pan_goal = random.uniform(-3.141/8, 3.141/8)
    head_roll_goal = random.uniform(-0.2, 0.2)
    head_tilt_goal = random.uniform(-0.8, -0.4)
    goal = [head_pan_goal, head_roll_goal, head_tilt_goal]
    start_time = rospy.get_time()
    start_position = current_pos    
    next_pos = [0, 0, 0]
    rospy.logwarn(goal)
    # move to that goal
    k = 0
    while goal_current_distance > precision:
        if rospy.is_shutdown():
            break
        if k > 100:
            break
        k += 1
        # compute how far we should have moved
        rad = (rospy.get_time() - start_time) * speed
        for i in range(0, 3):
            # move in direction of goal and stop at goal
            if goal[i] > start_position[i]:
                if (i == 0 or i == 1):
                    next_pos[i] = max(start_position[i] + rad, goal[i])
                else:
                    next_pos[i] = min(start_position[i] - rad, goal[i])
            else:
                if (i==0 or i ==1):
                    next_pos[i] = min(start_position[i] + rad, goal[i])
                else:
                    next_pos[i] = max(start_position[i] + rad, goal[i])
        msg.x = goal[0]
        msg.y = goal[1]
        msg.z = goal[2]
        joint_goal_publisher.publish(msg)
        rate.sleep()
    rospy.sleep(1.0)
    if goal_current_distance == -1:
        rospy.logwarn_throttle(1, "I wont start to move till I get joint states!")

