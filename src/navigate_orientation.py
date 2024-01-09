#!/usr/bin/env python2.7

'''
	This file is the whole navigation process for the orientation
	------------------------------------------------------------------
	To run this patrol process file only, you need to run the lines below in command prompt:
	- roslaunch jupiterobot_bringup jupiterobot_bringup.launch
	- roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/home/mustar/catkin_ws/src/orientation_robot/orientation_navigation/maps/fsktm_blockA_new_edited_1.yaml
	- roslaunch turtlebot_rviz_launchers view_navigation.launch
	- rosrun orientation_navigation navigate_orientation.py
'''

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from math import radians

original = 0

class FacultyTour:
	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		# ================== INITIALIZATION ================== 
		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server...")

		# Wait for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(120))
		rospy.loginfo("Connected to move base server")

		# A variable to hold the initial pose of the robot to be set by the user in RViz
		initial_pose = PoseWithCovarianceStamped()
		rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

		# Get the initial pose from the user
		rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
		rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

		# Make sure we have the initial pose
		while initial_pose.header.stamp == "":
			rospy.sleep(1)

		self.location = rospy.Publisher("location", String, queue_size=10)
		# =====================================================
			
		rospy.loginfo("Ready to go")
		rospy.sleep(1)

		self.last_location = ""

		# Coordinates for locations in FSKTM Block A
		# 1 -> BK1
		# 2 -> Office
		# 3 -> Cube
		# 4 -> Toilet
		locations = dict()
		locations['bk1'] = [4.74, 4.47, 0.68, 0.73]
		locations['toilet'] = [-0.41, 5.10, 1, 0.03]
		locations['office'] = [12.46, 3.56, -0.04, 1]
		locations['cube'] = [10.97, -8.50, 1, 0.02]

		# Start patrolling flow 
		self.goal = MoveBaseGoal()
		rospy.loginfo("Start Orientation")
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()
		rospy.sleep(2)

		while not rospy.is_shutdown():
			destination_msg = rospy.wait_for_message("location", String)
			destination_msg = destination_msg.data

			print("Destination:", destination_msg)
			if destination_msg == "no way":
				break

			waypoints = []
    
			if destination_msg == self.last_location:
				continue

			condition1 = (self.last_location == 'cube' and destination_msg == 'bk1') or (self.last_location == 'bk1' and destination_msg == 'cube') or (self.last_location == 'toilet' and destination_msg == 'cube')
			condition2 = (self.last_location == 'office' and destination_msg == 'toilet')
			condition3 = (self.last_location == '' and destination_msg == 'office')
			
			if condition1:
				waypoints.append(locations['office'])
				waypoints.append(locations[destination_msg])
				
			elif condition2:
				waypoints.append(locations['bk1'])
				waypoints.append(locations[destination_msg])
				
			elif condition3:
				waypoints.append(locations['toilet'])
				waypoints.append(locations[destination_msg])

			else:
				waypoints.append(locations[destination_msg])

			self.last_location = destination_msg
			
			for point in waypoints:
				destination = Pose(Point(point[0], point[1], 0.000), Quaternion(0.0, 0.0, point[2], point[3]))

				# Robot will go to point 1 by 1
				rospy.loginfo("Navigating...")
				rospy.sleep(2)
				self.goal.target_pose.pose = destination
				self.move_base.send_goal(self.goal)
				waiting = self.move_base.wait_for_result(rospy.Duration(300))
				print(waiting)
				if waiting == 1:
					rospy.sleep(2)
					continue
				else:
					continue
			rospy.loginfo("Reached " + destination_msg)


		rospy.loginfo("Going back initial point")
		# rospy.sleep(2)
		# initial_point = [-1.4702,0.0108,-0.04468, 0.9990]
		# self.goal.target_pose.pose = Pose(Point(initial_point[0], initial_point[1], 0.000), Quaternion(0.0, 0.0, initial_point[2], initial_point[3]))
		# self.move_base.send_goal(self.goal)
		# end_point = self.move_base.wait_for_result(rospy.Duration(300))
		# if end_point == 1:
		# 	rospy.loginfo("Reached initial point")
		# 	rospy.sleep(2)
		# 	self.cleanup()

	def update_initial_pose(self, initial_pose):
		global original
		self.initial_pose = initial_pose
		if original == 0:
			self.origin = self.initial_pose.pose.pose
			original = 1

	def cleanup(self):
		rospy.loginfo("Shutting down navigation	....")
		self.move_base.cancel_goal()

if __name__=="__main__":
	rospy.init_node('navigator')
	try:
		FacultyTour()
	except:
		pass
