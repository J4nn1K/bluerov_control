#!/usr/bin/env python

import rospy
import math
from control import pid
from std_msgs.msg import Float64
from bluerov_control.msg import Configuration, ControllerSetpoints


class SurgeControlNode(pid.PidNode):
    def __init__(self, name):
        super(SurgeControlNode, self).__init__(name=name)

        self.setpoint = 0.0
        self.controller_active = False

        self.controller_setpoint_sub = rospy.Subscriber("controller_setpoints",
                                                        ControllerSetpoints,
                                                        self.on_controller_setpoints)

        self.configuration_sub = rospy.Subscriber("configuration",
                                                  Configuration,
                                                  self.on_configuration,
                                                  queue_size=1)

        self.surge_pub = rospy.Publisher("surge",
                                         Float64,
                                         queue_size=1)

    def on_controller_setpoints(self, msg):
        self.controller_active = msg.surge_controller_setpoint

    def on_configuration(self, msg):
        now = msg.header.stamp.to_sec()
        distance = msg.distance
        
        if self.controller_active:
            with self.data_lock:
                error = -(self.setpoint - distance)
                u = self.update_controller(error=error, now=now)
        else:
            u = 0.0

        self.surge_pub.publish(Float64(u))


def main():
    node = SurgeControlNode("surge_controller")
    node.run()


if __name__ == "__main__":
    main()
