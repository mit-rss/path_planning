#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Point
import time, os
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray
import rospkg

class BuildTrajectory(object):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """
    def __init__(self):
        rospack = rospkg.RosPack()
        lab6_path = rospack.get_path("lab6")
        self.save_path = os.path.join(lab6_path+"/trajectories/", time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj") #%Y-%m-%d-%H-%M-%S
        self.trajectory = LineTrajectory("/built_trajectory")
        self.data_points = []
        self.count = 0
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.trajectory_points = rospy.Publisher("/traj_pts", Marker, queue_size=20)
        self.trajectory.publish_viz()

        # save the built trajectory on shutdown
        rospy.on_shutdown(self.saveTrajectory)

    def publish_trajectory(self):
        self.traj_pub.publish(self.trajectory.toPoseArray())

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)

    def clicked_pose(self,msg):
        self.count += 1
        point = Point()
        point.x = msg.point.x
        point.y = msg.point.y
        self.trajectory.addPoint(point)
        self.data_points.append(point)
        self.mark_pt(self.trajectory_points, (0,1,0), self.data_points)
        if self.count > 2:
            rospy.loginfo("PUBLISH TRAJ")
            self.publish_trajectory()
            self.trajectory.publish_viz()


    def mark_pt(self, subscriber, color_tup, data):
        mark_pt = Marker()
        mark_pt.header.frame_id = "/map"
        mark_pt.header.stamp = rospy.Time.now()
        mark_pt.type  = mark_pt.SPHERE_LIST
        mark_pt.action = mark_pt.ADD
        mark_pt.scale.x = .5
        mark_pt.scale.y = .5
        mark_pt.scale.z= .5
        mark_pt.color.a =1.0
        mark_pt.color.r=color_tup[0]
        mark_pt.color.g = color_tup[1]
        mark_pt.color.b = color_tup[2]
        mark_pt.points = data
        subscriber.publish(mark_pt)


if __name__=="__main__":
    rospy.init_node("build_trajectory")
    pf = BuildTrajectory()
    rospy.spin()
