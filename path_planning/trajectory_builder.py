#!/usr/bin/env python3

import os
import rclpy
import time
from geometry_msgs.msg import PointStamped, PoseArray, Point
from rclpy.node import Node
from typing import List, Tuple
from visualization_msgs.msg import Marker

from path_planning.utils import LineTrajectory


class BuildTrajectory(Node):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """

    def __init__(self):
        super().__init__("trajectory_builder")

        save_prefix = os.path.join(os.environ["HOME"], "lab6_trajectories")

        if not os.path.exists(save_prefix):
            self.get_logger().info("Creating the trajectory logging directory: {}".format(save_prefix))
            os.makedirs(save_prefix)

        self.save_path = os.path.join(save_prefix,
                                      time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj")

        self.trajectory = LineTrajectory(self, "/built_trajectory")
        self.data_points: List[Tuple[float, float]] = []
        self.count = 0
        self.click_sub = self.create_subscription(PointStamped, "/clicked_point", self.clicked_pose, 10)
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory/current", 10)
        self.trajectory_points = self.create_publisher(Marker, "/traj_pts", 20)
        self.trajectory.publish_viz()

        # save the built trajectory on shutdown
        self.get_logger().info("Press Ctrl+C to save the trajectory and exit.")

    def publish_trajectory(self):
        self.traj_pub.publish(self.trajectory.toPoseArray())

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)
        self.get_logger().info("Trajectory saved to: {}".format(self.save_path))

    def clicked_pose(self, msg):
        self.count += 1
        point = Point()
        point.x = msg.point.x
        point.y = msg.point.y
        self.trajectory.addPoint((point.x, point.y))
        self.data_points.append((point.x, point.y))
        self.mark_pt(self.trajectory_points, (0.0, 1.0, 0.0), self.data_points)
        if self.count > 2:
            self.get_logger().info("Publishing trajectory")
            self.publish_trajectory()
            self.trajectory.publish_viz()
            self.saveTrajectory()

    def tuple_to_point(self, data_points: List[Tuple[float, float]]) -> List[Point]:
        return [Point(x=x, y=y) for x, y in data_points]

    def mark_pt(self, subscriber, color_tup, data):
        msg_data = self.tuple_to_point(data)

        mark_pt = Marker()
        mark_pt.header.frame_id = "/map"
        mark_pt.header.stamp = self.get_clock().now().to_msg()
        mark_pt.type = mark_pt.SPHERE_LIST
        mark_pt.action = mark_pt.ADD
        mark_pt.scale.x = .5
        mark_pt.scale.y = .5
        mark_pt.scale.z = .5
        mark_pt.color.a = 1.0
        mark_pt.color.r = color_tup[0]
        mark_pt.color.g = color_tup[1]
        mark_pt.color.b = color_tup[2]
        mark_pt.points = msg_data
        subscriber.publish(mark_pt)


def main(args=None):
    rclpy.init(args=args)
    build_traj = BuildTrajectory()
    rclpy.spin(build_traj)
    build_traj.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
