#!/usr/bin/env python

import threading
import rospy
from hippocampus_common.node import Node
from bluerov_control.msg import Configuration, ControllerSetpoints


class TrajectoryGeneratorNode(Node):
    def __init__(self, name):
        super(TrajectoryGeneratorNode, self).__init__(name=name)
        self.data_lock = threading.RLock()
        
        self.yaw_setpoint = False
        self.sway_setpoint = False
        self.surge_setpoint = False
        
        # TODO in ControllerSetpoints.msg bool und value reinnehmen (z.B. fuer anderen Anfahrtswinkel / Distanz)
        
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
            if abs(body_to_LOS) < 0.05:
                self.sway_setpoint = True
            else: 
                self.sway_setpoint = False
            
            # start sway controller when vehicle orientation is correct
            if abs(body_to_LOS) < 0.05 and abs(object_to_LOS) < 0.05:
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
