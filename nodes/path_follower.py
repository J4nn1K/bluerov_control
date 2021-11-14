#!/usr/bin/env python

import rospy
import numpy as np
from hippocampus_common.node import Node
from bluerov_control.msg import PathTargets, ControllerErrors


class PathFollowerNode(Node):
    def __init__(self, name):
        super(PathFollowerNode, self).__init__(name)

        self.path_targets_sub = rospy.Subscriber("path_targets",
                                                 PathTargets,
                                                 self.on_targets,
                                                 queue_size=1)

        self.controller_errors_pub = rospy.Publisher("controller_errors",
                                                        ControllerErrors,
                                                        queue_size=1)

    def on_targets(self, msg):
        current_x = msg.current_x
        current_y = msg.current_y
        current_yaw_angle = msg.current_yaw
        target_x = msg.target_x
        target_y = msg.target_y

        # YAW CONTROL:
        # transform vehicle orientation (rotate +pi, map to [-pi,pi])
        transformed_current_yaw_angle = current_yaw_angle + np.pi
        if(transformed_current_yaw_angle > np.pi):
            transformed_current_yaw_angle -= 2*np.pi
        yaw_angle_to_object = np.arctan2(current_y, current_x)

        # Vektor ausgedrueckt in Objektsystem:
        vector_x = target_x - current_x
        vector_y = target_y - current_y
        # Vektor transformiert in Fahrzeugsystem:
        target_x_in_b = np.cos(current_yaw_angle)*vector_x + \
            np.sin(current_yaw_angle)*vector_y
        target_y_in_b = -np.sin(current_yaw_angle)*vector_x + \
            np.cos(current_yaw_angle)*vector_y
        
        yaw_error = yaw_angle_to_object - transformed_current_yaw_angle
        surge_error = target_x_in_b
        sway_error = target_y_in_b
        
        # yaw_angle_to_target = np.arctan2(vector_y, vector_x)
        # rospy.loginfo(180*yaw_angle_to_object/np.pi)
        
        
        
        if abs(yaw_error) < 0.1:
            sway_control = True
            surge_control = True
        else: 
            sway_control = False
            surge_control = False
            
        message = ControllerErrors()
        message.header.stamp = msg.header.stamp
        message.yaw_control = True
        message.sway_control = sway_control
        message.surge_control = surge_control
        message.yaw_error = yaw_error
        message.sway_error = sway_error
        message.surge_error = surge_error

        self.controller_errors_pub.publish(message)

def main():
    node = PathFollowerNode("path_follower")
    node.run()


if __name__ == "__main__":
    main()
