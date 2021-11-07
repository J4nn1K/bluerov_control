#!/usr/bin/env python

from __future__ import print_function
from matplotlib import pyplot as plt
import math
import rospy
from bluerov_control.srv import *


def bezier_client(x_pose_from_object, y_pose_from_object, yaw_angle_from_object):
    rospy.wait_for_service('generate_bezier_curve')
    try:
        generate_bezier_curve = rospy.ServiceProxy(
            'generate_bezier_curve', Bezier)

        resp1 = generate_bezier_curve(
            x_pose_from_object, y_pose_from_object, yaw_angle_from_object)
        return resp1
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    x = 1
    y = 0.5
    yaw = 0.9*math.pi
    
    response = bezier_client(x, y, yaw)
    x_waypoints = response.x_waypoints
    x_waypoints = x_waypoints[::-1]
    y_waypoints = response.y_waypoints
    y_waypoints = y_waypoints[::-1]

    print(x_waypoints)
    
    plt.plot(x_waypoints, y_waypoints,".")
    plt.show()