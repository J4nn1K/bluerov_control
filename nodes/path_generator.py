#!/usr/bin/env python

from __future__ import print_function
import rospy
import math
import numpy as np
from scipy.special import comb
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt
from bluerov_control.path import Path
from bluerov_control.msg import PathTargets

class PathGenerator():
    def __init__(self, name):
        rospy.init_node(name)

        self.path_targets_pub = rospy.Publisher("path_targets", PathTargets, queue_size=1)

        self.use_ground_truth = rospy.get_param("~use_ground_truth",
                                                default=False)
        if self.use_ground_truth:
            self.msg = rospy.wait_for_message("pose_from_object_ground_truth", PoseStamped)
            self.pose_ground_truth_sub = rospy.Subscriber("pose_from_object_ground_truth",
                                                          PoseStamped,
                                                          self.on_pose,
                                                          queue_size=1)
        else:
            self.msg = rospy.wait_for_message("pose_from_object", PoseStamped)
            self.pose_sub = rospy.Subscriber("pose_from_object",
                                             PoseStamped,
                                             self.on_pose,
                                             queue_size=1)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw_angle = 0.0
        self.position = np.array([0]*2, dtype=float)

        self.offset_distance_body = 0.0
        self.offset_distance_object = 1.6

        self.look_ahead_distance = 0.1

    def get_control_points(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        quat = msg.pose.orientation
        orientation = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw_angle = orientation[2]

        point0_x = x
        point0_y = y
        point1_x = x + math.cos(yaw_angle)*self.offset_distance_body
        point1_y = y + math.sin(yaw_angle)*self.offset_distance_body
        point2_x = self.offset_distance_object
        point2_y = 0.0
        point3_x = 0.38
        point3_y = 0.0

        self.points = [
            [point0_x, point0_y],
            [point1_x, point1_y],
            [point2_x, point2_y],
            [point3_x, point3_y]
        ]

    def bernstein_polynomial(self, i, n, t):
        return comb(n, i) * (t**(n-i)) * (1 - t)**i

    def bezier_curve(self, points, n):

        nPoints = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])

        t = np.linspace(0.0, 1.0, n)

        polynomial_array = np.array([self.bernstein_polynomial(
            i, nPoints-1, t) for i in range(0, nPoints)])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return xvals, yvals

    def plot_curve(self, x_waypoints, y_waypoints):
        plt.plot(x_waypoints, y_waypoints, ".")
        plt.plot([self.points[0][0], self.points[1][0]], [
                 self.points[0][1], self.points[1][1]], "ro-")
        plt.plot([self.points[2][0], self.points[3][0]], [
                 self.points[2][1], self.points[3][1]], "ro-")
        plt.xlim([-0.1, 3])
        plt.xlabel("$x_o$")
        plt.ylim([-0.8, 0.8])
        plt.ylabel("$y_o$")
        plt.rcParams.update({"text.usetex": True})
        plt.show()

    def on_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        quat = msg.pose.orientation
        orientation = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.yaw_angle = orientation[2]

        self.position = np.array([self.x, self.y])

    def run(self):
        self.get_control_points(self.msg)

        x_waypoints, y_waypoints = self.bezier_curve(self.points, 200)
        x_waypoints = x_waypoints[::-1]
        y_waypoints = y_waypoints[::-1]
        waypoints = np.array([x_waypoints, y_waypoints])

        self.plot_curve(x_waypoints, y_waypoints)

        self.path = Path(waypoints)
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.path.update_target(position=self.position, look_ahead_distance=self.look_ahead_distance):
                target = self.path.get_target_point()
                
                msg = PathTargets()
                msg.header.stamp = rospy.Time.now()
                msg.current_x = self.x
                msg.current_y = self.y
                msg.current_yaw = self.yaw_angle
                msg.target_x = target[0]
                msg.target_y = target[1]

                self.path_targets_pub.publish(msg)
                
                #vector = target - self.position
                #yaw_angle_to_target = math.atan2(vector[1], vector[0])

            else:
                rospy.logwarn("Could not update target position.")

            rate.sleep()

def main():
    node = PathGenerator("path_generator")
    node.run()


if __name__ == "__main__":
    main()
