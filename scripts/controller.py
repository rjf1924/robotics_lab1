#!/usr/bin/env python3

import rospy

from turtlesim.msg import Pose

import math

from geometry_msgs.msg import Twist

from robotics_lab1.msg import Turtlecontrol

pos_msg = Pose()

ctrl_msg = Turtlecontrol()

def pose_callback(data):
    global pos_msg
    pos_msg = data

def control_callback(data):
    global ctrl_msg
    ctrl_msg = data

if __name__ == '__main__':
    rospy.init_node("turtle_controller", anonymous = True)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose,pose_callback)
    control_subscriber = rospy.Subscriber('/turtle1/control_params',Turtlecontrol,control_callback)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    loop_rate = rospy.Rate(10)
    
    vel_msg = Twist()

    while not rospy.is_shutdown():
        vel_msg.linear.x = ctrl_msg.kp * (ctrl_msg.xd - pos_msg.x)
        print(ctrl_msg.xd)
        print(ctrl_msg.kp)
        print(ctrl_msg.xd - pos_msg.x)
        print("")
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

