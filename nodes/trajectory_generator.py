#!/usr/bin/env python

import threading
import rospy
import math
from hippocampus_common.node import Node
from bluerov_control.msg import Configuration, ControllerSetpoints

# 4D Trajectory Sequencer
class TrajectoryGeneratorNode(Node):
    def __init__(self, name):
        super(TrajectoryGeneratorNode, self).__init__(name=name)
        self.data_lock = threading.RLock()
        
        self.heave_control = False
        self.yaw_control = False
        self.sway_control = False  
        self.surge_control = False
        
        self.heave_setpoint = -0.53
        self.yaw_setpoint = 0.0
        self.sway_setpoint = 0.0
        self.surge_setpoint = 0.0

        self.heave_tolerance = 0.1
        self.yaw_tolerance = 0.1
        self.sway_tolerance = 0.1

        
        self.configuration_sub = rospy.Subscriber("configuration",
                                                  Configuration,
                                                  self.on_configuration,
                                                  queue_size=1)

        self.controller_setpoint_pub = rospy.Publisher("controller_setpoints",
                                                       ControllerSetpoints,
                                                       queue_size=1)

    def on_configuration(self, msg):
        with self.data_lock:
            # configuration
            # TODO depth bzw. heave hinzufuegen
            body_to_LOS = msg.body_to_LOS
            object_to_LOS = msg.object_to_LOS
            distance = msg.distance

            # start heave control
            #self.heave_control = True
            
            
            # start yaw controller
            self.yaw_control = True
            
            # start sway controller when vehicle orientation is correct
            if abs(body_to_LOS) < self.yaw_tolerance:
                self.sway_control = True
            else: 
                self.sway_control = False
            
            # start surge controller when vehicle position is correct
            if abs(body_to_LOS) < self.yaw_tolerance and abs(object_to_LOS) < self.sway_tolerance:
                self.surge_control = True
            else:
                self.surge_control = False

            setpoints = ControllerSetpoints()
            setpoints.yaw_control = self.yaw_control
            setpoints.sway_control = self.sway_control 
            setpoints.surge_control = self.surge_control
            setpoints.yaw_setpoint = self.yaw_setpoint
            setpoints.sway_setpoint = self.sway_setpoint
            setpoints.surge_setpoint = self.surge_setpoint

            self.controller_setpoint_pub.publish(setpoints)

def main():
    node = TrajectoryGeneratorNode("trajectory_generator")
    node.run()


if __name__ == "__main__":
    main()
