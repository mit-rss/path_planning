import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray,Pose

from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

from .utils import LineTrajectory


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 1.2  # FILL IN #
        self.get_logger().info(f'{self.lookahead}')
        self.speed = 1.  # FILL IN #
        self.wheelbase_length = 0.3  # FILL IN #

        self.trajectory = LineTrajectory("/followed_trajectory")

        self.traj_sub = self.create_subscription(PoseArray,"/trajectory/current", self.trajectory_callback, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        
        #subscribe to particle filter localization #turn back on for real car
        #self.pose_sub = self.create_subscription(Odometry,"/pf/pose/odom",self.pose_callback, 1)
        self.pose_sub = self.create_subscription(Odometry,"/odom",self.pose_callback, 1)   

        #viz target point
        self.viz_pub = self.create_publisher(PoseArray, "/target_point", 1) 

        #viz p1 point
        self.viz_pubp1 = self.create_publisher(PoseArray, "/p1", 1)

        #viz p2 point
        self.viz_pubp2 = self.create_publisher(PoseArray, "/p2", 1)


    def pose_callback(self, odometry_msg):
        car_x = odometry_msg.pose.pose.position.x
        car_y = odometry_msg.pose.pose.position.y
        car_z = odometry_msg.pose.pose.position.z
        car_xy_pos = np.array((car_x,car_y))
        

        car_ort_x = odometry_msg.pose.pose.orientation.x
        car_ort_y = odometry_msg.pose.pose.orientation.y
        car_ort_z = odometry_msg.pose.pose.orientation.z
        car_ort_w = odometry_msg.pose.pose.orientation.w

        #if there is no trajectory loaded wait
        #self.get_logger().info(f"{np.array(self.trajectory.points)}")
        if np.array(self.trajectory.points).size == 0:
            self.get_logger().info(f"waiting for trajectory")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0 
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        #find the segment that is nearest to the car
        traj_points = np.array(self.trajectory.points)
        N=traj_points.shape[0]
        nrst_distances = self.find_dist(traj_points[0:N-1,:],traj_points[1:N,:],car_xy_pos)
        #self.get_logger().info(f'nrst_dist {nrst_distances}')
        min_index = np.argmin(nrst_distances)
        #self.get_logger().info(f'min index{min_index}')
        #self.get_logger().info(f'index {min_index}')
        nearest_segment = traj_points[min_index:(min_index+2),:]
        

        #find the intersection point(s) of the segment with the circle
        #self.get_logger().info(f'nearest segment{nearest_segment}')
        p1 = nearest_segment[0,:]
        p2 = nearest_segment[1,:]
    
        #if the car is so close to the end that it cant make a 90 degree turn start looking at the next segment
        dist_from_end = np.linalg.norm(car_xy_pos-p2)
        i_counter = 0
        while (dist_from_end <= self.lookahead) and (min_index+i_counter+3) <= traj_points.shape[0]:
            nearest_segment = traj_points[(min_index+i_counter+1):(min_index+i_counter+3),:]
            p1 = nearest_segment[0,:]
            p2 = nearest_segment[1,:]
            dist_from_end = np.linalg.norm(car_xy_pos-p2)
            i_counter+=1

        V = p2-p1
        r = self.lookahead
        Q = car_xy_pos

        a = np.dot(V,V)
        b = 2 * np.dot(V,(p1 - Q))
        c = np.dot(p1,p1) + np.dot(Q,Q) - 2 * np.dot(p1,Q) - r**2

        disc = b**2 - 4 * a * c
        if disc < 0: #no solution
            #self.get_logger().info(f'no soln')
            return None
        
        # if a solution exists find it
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a) #keep this in case we need

        #in the event that the line does intersect find the points
        int1 = p1 + t1*V
        int2 = p1 + t2*V
        
        #now pick the point that is in front of the car
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            #if the circle would only intersect if the line was extened make the end point be the goal
            target_point = p2
        else:
            #i think you just want to pick the one that is closer to the second point, ie the greatest t?
            #t1 should be greater, but it might be out of range! if it is out of range then use t2
            target_point = int1
        
         
        
        
        
        

        #visualize the target_point
        from threading import Lock
        self.lock = Lock()
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(PurePursuit.pose_to_msg(target_point[0], target_point[1], 0.0))
        self.viz_pub.publish(msg)

         #visualize the p1
        from threading import Lock
        self.lock = Lock()
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(PurePursuit.pose_to_msg(p1[0], p1[1], 0.0))
        self.viz_pubp1.publish(msg)

         #visualize the target_point
        from threading import Lock
        self.lock = Lock()
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(PurePursuit.pose_to_msg(p2[0], p2[1], 0.0))
        self.viz_pubp2.publish(msg)
        


        
        
        #now that we  have the target point we can do the pure pursuite algorithm
        #determine the heading of the car in the world frame
        theta = euler_from_quaternion((car_ort_x, car_ort_y, car_ort_z, car_ort_w))[-1]
        car_x_direction = np.array([math.cos(theta),math.sin(theta)])
        car_y_direction = np.array([-math.sin(theta),math.cos(theta)])
        
        #calculate the position of the point relative to the car
        target_point_point_rel_to_car = car_xy_pos-target_point
        
        #in order to get the correct eta we need to broadcast these to the cars frame
        rel_x = np.dot(car_x_direction,target_point_point_rel_to_car)
        rel_y = np.dot(car_y_direction,target_point_point_rel_to_car)
        eta = math.atan(rel_y/rel_x)

        #calculate the steering angle  (right is negative left is postive)
        steering_angle = math.atan((2*self.wheelbase_length*math.sin(eta))/self.lookahead)
        steering_angle = np.clip(np.array([steering_angle]),-0.34,0.34)[0]


        #publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed 
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)



    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True

    def find_dist(self, linepoint1, linepoint2, point):
        linepoint1 = np.array(linepoint1)
        linepoint2 = np.array(linepoint2)
        point = np.array(point)
        delta = linepoint2-linepoint1
        norm_vec = np.empty(delta.shape)
        norm_vec[:,0] = -delta[:,1]
        norm_vec[:,1] = delta[:,0]
        qp = point-linepoint1
        norm_mag = np.sum((norm_vec*norm_vec),axis = 1)**.5
        # Parameter t for the projection of the point onto the line segment
        t = np.sum(qp*delta,axis=1)/np.sum(delta*delta,axis=1)
        print(f't: {t}')
        #this is essentially doing the dot product, gives shortest distance to infinite line
        d = np.abs(np.sum(norm_vec*qp,axis=1))/norm_mag 
        #the edge case
        d = np.where(t>=1, np.linalg.norm(linepoint2-point, axis=1),d)
        d = np.where(t<=0, np.linalg.norm(linepoint1-point, axis=1),d)
        #self.get_logger().info(f'd: {d}')
        return d
        
    @staticmethod
    def pose_to_msg(x, y, theta):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = 0.0
        
        quaternion = quaternion_from_euler(0.0, 0.0, theta)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        return msg


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
