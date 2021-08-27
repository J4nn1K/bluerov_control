#!/usr/bin/env python

import threading
import rospy
import math
from hippocampus_common.node import Node
from bluerov_control.msg import Configuration, ControllerSetpoints


class TrajectoryGeneratorNode(Node):
    def __init__(self, name):
        super(TrajectoryGeneratorNode, self).__init__(name=name)
        self.data_lock = threading.RLock()
        
        # TODO in ControllerSetpoints.msg bool und value reinnehmen (z.B. fuer anderen Anfahrtswinkel / Distanz)
        # self.yaw_setpoint_active = False
        # self.sway_setpoint_active = False
        # self.surge_setpoint_active = False
        self.yaw_setpoint = False
        self.sway_setpoint = False
        self.surge_setpoint = False

        self.error_tolerance = 0.1*math.pi
        
        self.configuration_sub = rospy.Subscriber("configuration",
                                                  Configuration,
                                                  self.on_configuration,
                                                  queue_size=1)

        self.controller_setpoint_pub = rospy.Publisher("controller_setpoints",
                                                       ControllerSetpoints,
                                                       queue_size=1)

    def on_configuration(self, msg):
        with self.data_lock:
            body_to_LOS = msg.body_to_LOS
            object_to_LOS = msg.object_to_LOS
            distance = msg.distance

            # start yaw controller
            self.yaw_setpoint = True
            
            # start sway controller when vehicle orientation is correct
            if abs(body_to_LOS) < self.error_tolerance:
                self.sway_setpoint = True
            else: 
                self.sway_setpoint = False
            
            # start surge controller when vehicle position is correct
            if abs(body_to_LOS) < self.error_tolerance and abs(object_to_LOS) < self.error_tolerance:
                self.surge_setpoint = True
            else:
                self.surge_setpoint = False

            setpoints = ControllerSetpoints()
            setpoints.yaw_controller_setpoint = self.yaw_setpoint
            setpoints.sway_controller_setpoint = self.sway_setpoint
            setpoints.surge_controller_setpoint = self.surge_setpoint

            self.controller_setpoint_pub.publish(setpoints)

def main():
    node = TrajectoryGeneratorNode("trajectory_generator")
    node.run()


if __name__ == "__main__":
    main()