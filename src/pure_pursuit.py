#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = rospy.get_param("lookahead",0.5)
        self.speed            = rospy.get_param("speed",1.0)
        #self.wrap             = # FILL IN #
        self.wheelbase_length = 0.568
        self.base_link_offset = rospy.get_param("base_link_offset",0.1)
        self.robot_pose = None
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/pf/pose/odom", Odometry, self.pose_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
    
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        
        #
        # Find nearest line segment
        #
        segments = np.array(self.trajectory.points())
        num_segments = segments.shape[0]
        robot_point = np.tile(self.robot_pose[:2],(num_segments-1,1))
        
        segment_start = segments[:-1,:]
        segment_end = segments[1:,:]
        
        #Vectorize
        robot_vector = robot_point-segment_start
        segment_vector = segment_end-segment_start

        #Normalize and Scale
        segment_length = np.linalg.norm(segment_vector, axis=1)
        segment_length = np.reshape(segment_length, (num_segments-1,1))

        unit_segment = np.divide(segment_vector,segment_length)
        scaled_robot_point = np.divide(robot_vector,segment_length)

        #Project
        scaled_dot_product = np.sum(unit_segment*scaled_robot_point, axis=1)

        #Clip and Rescale
        scaled_dot_product = np.clip(scaled_dot_product,0.0,1.0)
        scaled_dot_product = np.reshape(scaled_dot_product, (num_segments-1,1))
        
        scaled_segment = segment_vector*scaled_dot_product
        
        #Offset
        shifted = np.add(scaled_segment,segment_start)
        
        #Find Distances
        shortest_distance = np.linalg.norm(shifted-robot_point, axis=1)
        
        #Grab relevant segment
        segment_index = np.argmin(shortest_distance)
        nearest_segment_start = segments[segment_index]
        nearest_segment_end = segments[segment_index+1]
        
     def pose_callback(self, msg):
         ''' Clears the currently followed trajectory, and loads the new one from the message
         '''
         pose = [msg.pose.pose.position.x+self.base_link_offset,msg.pose.pose.position.y]
         r = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
         theta = r.as_euler('XYZ')[2]
         pose.append(theta)
         self.robot_pose = np.array(pose)
        

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
