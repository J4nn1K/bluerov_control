#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.special import comb
from bluerov_control.srv import Bezier, BezierResponse


def bernstein_polynomial(i, n, t):
    return comb(n, i) * (t**(n-i)) * (1 - t)**i


def bezier_curve(points, nTimes=1000):
    """
        points should be a list of lists, or list of tuples
        such as [ [1,1], 
                  [2,3], 
                  [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([bernstein_polynomial(
        i, nPoints-1, t) for i in range(0, nPoints)])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals


def handle_generate_bezier_curve(req):
    offset_distance_body = 0.5
    offset_distance_object = 0.5

    point0_x = req.x_pose_from_object
    point0_y = req.y_pose_from_object
    point1_x = req.x_pose_from_object + \
        math.cos(req.yaw_angle_from_object)*offset_distance_body
    point1_y = req.y_pose_from_object + \
        math.sin(req.yaw_angle_from_object)*offset_distance_body
    point2_x = offset_distance_object
    point2_y = 0.0
    point3_x = 0.0
    point3_y = 0.0

    points = [
        [point0_x, point0_y],
        [point1_x, point1_y],
        [point2_x, point2_y],
        [point3_x, point3_y]
    ]

    bezier_curve(points, 25)

    return BezierResponse(xvals, yvals)


def bezier_generator_server():
    rospy.init_node('bezier_generator_server')
    s = rospy.Service('generate_bezier_curve', Bezier,
                      handle_generate_bezier_curve)
    rospy.spin()


if __name__ == "__main__":
    bezier_generator_server()
