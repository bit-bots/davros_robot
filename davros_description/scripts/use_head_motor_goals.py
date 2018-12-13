#!/usr/bin/env python3
import rospy 
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Vector3

rospy.init_node("head_behav_to_davros")
publisher = rospy.Publisher("/davros_motor_goals", Vector3, queue_size=1)

def cb(msg):
    positions = msg.points[0].positions
    out_msg = Vector3()
    i = 0
    for name in msg.joint_names:
        if name == "RShoulderPitch":            
            out_msg.x = positions[i]
        elif name == "LShoulderPitch":
            out_msg.y = positions[i]
        if name == "RShoulderRoll":
            out_msg.z = positions[i]
        else:
            rospy.logerr("Joint names do not match")
        i += 1    
    publisher.publish(out_msg)

rospy.Subscriber("/head_motor_goals", JointTrajectory, cb)

while not rospy.is_shutdown():
    rospy.sleep(1)