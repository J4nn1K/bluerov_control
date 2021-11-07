#!/usr/bin/env python

from __future__ import print_function
import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from bluerov_control.srv import *
from matplotlib import pyplot as plt
import numpy as np
from scipy.special import comb



class PathGenerator():
    def __init__(self, name):
        rospy.init_node(name)

        self.pose_from_object_sub = rospy.Subscriber("pose_from_object",
                                                     PoseStamped,
                                                     self.on_pose,
                                                     queue_size=1)

        self.x = 0.0
        self.y = 0.0
        self.yaw_angle = 0.0
        
        self.offset_distance_body = 0.5
        self.offset_distance_object = 0.5

    def on_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        quat = msg.pose.orientation
        orientation = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.yaw_angle = orientation[2]

    # def bezier_client(self):
    #     rospy.wait_for_service('generate_bezier_curve')
    #     try:
    #         generate_bezier_curve = rospy.ServiceProxy(
    #             'generate_bezier_curve', Bezier)

    #         resp1 = generate_bezier_curve(self.x, self.y, self.yaw_angle)
    #         return resp1

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s" % e)
    
    def bernstein_polynomial(self, i, n, t):
        return comb(n, i) * (t**(n-i)) * (1 - t)**i


    def bezier_curve(self, points, nTimes=1000):

        nPoints = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])

        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([self.bernstein_polynomial(
            i, nPoints-1, t) for i in range(0, nPoints)])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return xvals, yvals


    def run(self):
        rospy.sleep(2)
        # response = self.bezier_client()

        point0_x = self.x
        point0_y = self.y
        point1_x = self.x + math.cos(self.yaw_angle)*self.offset_distance_body
        point1_y = self.y + math.sin(self.yaw_angle)*self.offset_distance_body
        point2_x = self.offset_distance_object
        point2_y = 0.0
        point3_x = 0.0
        point3_y = 0.0

        points = [
            [point0_x, point0_y],
            [point1_x, point1_y],
            [point2_x, point2_y],
            [point3_x, point3_y]
        ]

        x_waypoints, y_waypoints = self.bezier_curve(points, 25)
        
        x_waypoints = x_waypoints[::-1]
        y_waypoints = y_waypoints[::-1]

        ### OUTPUT ###

        for waypoint in x_waypoints:
            print("{:5.2f}".format(waypoint), end ='|')
        print(end ='\n')
        for waypoint in y_waypoints:
            print("{:5.2f}".format(waypoint), end ='|')
        print(end ='\n')
        
        plt.plot(x_waypoints, y_waypoints,".")
        plt.xlim([-0.1, 3])
        plt.ylim([-0.8, 0.8])
        plt.show()

def main():
    node = PathGenerator("path_generator")
    node.run()


if __name__ == "__main__":
    main()
