#!/usr/bin/env python

import rospy
import math
from control import pid
from std_msgs.msg import Float64
from bluerov_control.msg import ControllerErrors


class YawControlNode(pid.PidNode):
    def __init__(self, name):
        super(YawControlNode, self).__init__(name=name)

        self.error = 0.0
        self.controller_active = False

        self.controller_errors_sub = rospy.Subscriber("controller_errors",
                                                        ControllerErrors,
                                                        self.on_controller_errors)

        self.yaw_pub = rospy.Publisher("yaw",
                                       Float64,
                                       queue_size=1)

    def on_controller_errors(self, msg):
        self.controller_active = msg.yaw_control
        self.error = msg.yaw_error

        now = msg.header.stamp.to_sec()
        
        if self.controller_active:
            with self.data_lock:
                u = self.update_controller(error=self.error, now=now)
        else:
            u = 0.0

        self.yaw_pub.publish(Float64(u))
        


def main():
    node = YawControlNode("yaw_controller")
    node.run()


if __name__ == "__main__":
    main()
