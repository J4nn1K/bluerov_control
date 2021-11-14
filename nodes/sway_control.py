#!/usr/bin/env python

import rospy
import math
from control import pid
from std_msgs.msg import Float64
from bluerov_control.msg import ControllerErrors


class SwayControlNode(pid.PidNode):
    def __init__(self, name):
        super(SwayControlNode, self).__init__(name=name)

        self.error = 0.0
        self.controller_active = False

        self.controller_setpoint_sub = rospy.Subscriber("controller_errors",
                                                        ControllerErrors,
                                                        self.on_controller_errors)
        self.sway_pub = rospy.Publisher("sway",
                                        Float64,
                                        queue_size=1)

    def on_controller_errors(self, msg):
        self.controller_active = msg.sway_control
        self.error = msg.sway_error

        now = msg.header.stamp.to_sec()
        
        if self.controller_active:
            with self.data_lock:
               u = self.update_controller(error=self.error, now=now)
        else:
            u = 0.0
        
        self.sway_pub.publish(Float64(u))


def main():
    node = SwayControlNode("sway_controller")
    node.run()


if __name__ == "__main__":
    main()
