import rospy
import numpy as np
from yaml import load
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, Point32, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid
import json
import tf.transformations
import tf
from scipy import ndimage


EPSILON = 0.00000000001

''' These data structures can be used in the search function
'''

class LineTrajectory(object):
    """ A class to wrap and work with piecewise linear trajectories. """
    def __init__(self, viz_namespace=None):
        self.points = []
        self.distances = []
        self.has_acceleration = False
        self.visualize = False
        self.viz_namespace = viz_namespace

        if viz_namespace:
            self.visualize = True
            self.start_pub = rospy.Publisher(viz_namespace + "/start_point", Marker, queue_size = 1)
            self.traj_pub  = rospy.Publisher(viz_namespace + "/path", Marker, queue_size = 1)
            self.end_pub   = rospy.Publisher(viz_namespace + "/end_pose", Marker, queue_size = 1)

    # compute the distances along the path for all path segments beyond those already computed
    def update_distances(self):
        num_distances = len(self.distances)
        num_points = len(self.points)

        for i in xrange(num_distances,num_points):
            if i == 0:
                self.distances.append(0)
            else:
                p0 = self.points[i-1]
                p1 = self.points[i]
                delta = np.array([p0[0]-p1[0],p0[1]-p1[1]])
                self.distances.append(self.distances[i-1] + np.linalg.norm(delta))

    def distance_to_end(self, t):
        if not len(self.points) == len(self.distances):
            print "WARNING: Different number of distances and points, this should never happen! Expect incorrect results. See LineTrajectory class."
        dat = self.distance_along_trajectory(t)
        if dat == None:
            return None
        else:
            return self.distances[-1] - dat

    def distance_along_trajectory(self, t):
        # compute distance along path
        # ensure path boundaries are respected
        if t < 0 or t > len(self.points) - 1.0:
            return None
        i = int(t) # which segment
        t = t % 1.0 # how far along segment
        if t < EPSILON:
            return self.distances[i]
        else:
            return (1.0-t)*self.distances[i] + t*self.distances[i+1]

    def addPoint(self, point):
        print "adding point to trajectory:", point.x, point.y
        self.points.append((point.x, point.y))
        self.update_distances()
        self.mark_dirty()

    def clear(self):
        self.points = []
        self.distances = []
        self.mark_dirty()

    def empty(self):
        return len(self.points) == 0

    def save(self, path):
        print "Saving trajectory to:", path
        data = {}
        data["points"] = []
        for p in self.points:
            data["points"].append({"x": p[0], "y": p[1]})
        with open(path, 'w') as outfile:
            json.dump(data, outfile)

    def mark_dirty(self):
        self.has_acceleration = False

    def dirty(self):
        return not self.has_acceleration

    def load(self, path):
        print "Loading trajectory:", path
        with open(path) as json_file:
            json_data = json.load(json_file)
            for p in json_data["points"]:
                self.points.append((p["x"], p["y"]))
        self.update_distances()
        print "Loaded:", len(self.points), "points"
        self.mark_dirty()

    # build a trajectory class instance from a trajectory message
    def fromPoseArray(self, trajMsg):
        for p in trajMsg.poses:
            self.points.append((p.position.x, p.position.y))
        self.update_distances()
        self.mark_dirty()
        print "Loaded new trajectory with:", len(self.points), "points"

    def toPoseArray(self):
        traj = PoseArray()
        traj.header = self.make_header("/map")
        for i in xrange(len(self.points)):
            p = self.points[i]
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            traj.poses.append(pose)
        return traj

    def publish_start_point(self, duration=0.0, scale=0.1):
        should_publish = len(self.points) > 0
        if self.visualize and self.start_pub.get_num_connections() > 0:
            print "Publishing start point"
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 0
            marker.type = 2 # sphere
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[0][0]
                marker.pose.position.y = self.points[0][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.start_pub.publish(marker)
        # elif self.start_pub.get_num_connections() == 0:
        #   print "Not publishing start point, no subscribers"

    def publish_end_point(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.end_pub.get_num_connections() > 0:
            print "Publishing end point"
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 1
            marker.type = 2 # sphere
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[-1][0]
                marker.pose.position.y = self.points[-1][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.end_pub.publish(marker)
        # elif self.end_pub.get_num_connections() == 0:
        #   print "Not publishing end point, no subscribers"

    def publish_trajectory(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.traj_pub.get_num_connections() > 0:
            print "Publishing trajectory"
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 2
            marker.type = marker.LINE_STRIP # line strip
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = marker.ADD
                marker.scale.x = 0.3
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                for p in self.points:
                    pt = Point32()
                    pt.x = p[0]
                    pt.y = p[1]
                    pt.z = 0.0
                    marker.points.append(pt)
            else:
                # delete
                marker.action = marker.DELETE
            self.traj_pub.publish(marker)
            print('publishing traj')
        elif self.traj_pub.get_num_connections() == 0:
            print "Not publishing trajectory, no subscribers"

    def publish_viz(self, duration=0):
        if not self.visualize:
            print "Cannot visualize path, not initialized with visualization enabled"
            return

        self.publish_start_point(duration=duration)
        self.publish_trajectory(duration=duration)
        self.publish_end_point(duration=duration)

    def make_header(self, frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header
