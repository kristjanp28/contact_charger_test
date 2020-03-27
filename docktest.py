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
twist = Twist()
speed = 0.1
x = 0
y = 0
z = 0
th = 0
turn = 0

rospy.init_node('test_dock')

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
docking = rospy.ServiceProxy('dock', Dock)
client.wait_for_server()

def move(x, y, z, th):
	twist = Twist()
	twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
	pub.publish(twist)
	return

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
	x = random.uniform(-1,-0.5)
	y = random.uniform(-0.5,0.5)
	rot = 0
	print(x,y)
	move_to_pos(x,y,rot)


rospy.Subscriber('battery_state', BatteryState, bat_stat)

i = time.time() + 4
while time.time() < i:
	move(-1,0,0,0)
else:
	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
	pub.publish(twist)


random_position()
print("battery voltage: " + str(b_state.voltage))

