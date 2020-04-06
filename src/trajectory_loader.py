#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
import time, os
from utils import LineTrajectory

class LoadTrajectory(object):
    """ Loads a trajectory from the file system and publishes it to a ROS topic.
    """
    def __init__(self):
        self.path           = rospy.get_param("~trajectory")

        # initialize and load the trajectory
        self.trajectory = LineTrajectory("/loaded_trajectory")
        self.trajectory.load(self.path)

        self.pub_topic = "/trajectory/current"
        self.traj_pub = rospy.Publisher(self.pub_topic, PoseArray, queue_size=1)

        # need to wait a short period of time before publishing the first message
        time.sleep(0.5)

        # visualize the loaded trajectory
        self.trajectory.publish_viz()

        # send the trajectory
        self.publish_trajectory()


    def publish_trajectory(self):
        print "Publishing trajectory to:", self.pub_topic
        self.traj_pub.publish(self.trajectory.toPoseArray())

if __name__=="__main__":
    rospy.init_node("load_trajectory")
    pf = LoadTrajectory()
    rospy.spin()
