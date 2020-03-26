#!/usr/bin/env python

import rospy
import sys
import actionlib

import math
import time
import random

from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from sensor_msgs.msg import BatteryState
from docking.srv import *

b_state = BatteryState()



rospy.init_node('test_dock')

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
docking = rospy.ServiceProxy('dock', Dock)
client.wait_for_server()


def bat_stat(data):
	global b_state
	b_state = data

def move_to_pos(X, Y, Rotation):
	pose = PoseStamped()
	pose.header.frame_id = "base_link"
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = math.radians(Rotation)
	pose.pose.orientation.w = 1
	pose.pose.position.x = X
	pose.pose.position.y = Y
	goal = MoveBaseGoal()
	goal.target_pose = pose

	client.send_goal(goal)

def random_position():
	x = random.uniform(-0.5,-0.1)
	y = random.uniform(-0.5,0.5)
	rot = 0
	print(x,y)
	move_to_pos(x,y,rot)


rospy.Subscriber('battery_state', BatteryState, bat_stat)


move_to_pos(-0.2,0,0)
time.sleep(2)
move_to_pos(-0.2,0,0)
random_position()
print("battery voltage: " + str(b_state.voltage))
time.sleep(15)
docking(42, "-.8 -.10 0 0, -.4 -.10 0 0, -.17 -.10 0 0")

