#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils

from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

class PurePursuit(object):
	""" Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
	"""
	def __init__(self):
		self.trajectory_topic = rospy.get_param("~trajectory_topic")
		self.odom_topic       = rospy.get_param("~odom_topic")
		self.lookahead        = rospy.get_param("~lookahead")
		self.speed            = float(rospy.get_param("~speed"))
		self.wrap             = bool(rospy.get_param("~wrap"))
		wheelbase_length      = float(rospy.get_param("~wheelbase"))
		self.drive_topic      = rospy.get_param("~drive_topic")

		self.trajectory  = utils.LineTrajectory("/followed_trajectory")
		self.traj_sub = rospy.Subscriber(self.trajectory_topic, PolygonStamped, self.trajectory_callback, queue_size=1)

		'''
		Insert code here
		'''

	def trajectory_callback(self, msg):
		''' Clears the currently followed trajectory, and loads the new one from the message
		'''
		print "Receiving new trajectory:", len(msg.polygon.points), "points" 
		self.trajectory.clear()
		self.trajectory.fromPolygon(msg.polygon)
		self.trajectory.publish_viz(duration=0.0)

if __name__=="__main__":
	rospy.init_node("pure_pursuit")
	pf = PurePursuit()
	rospy.spin()
