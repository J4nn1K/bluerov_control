#!/usr/bin/env python

import rospy
import math
from control import pid
from std_msgs.msg import Float64
from bluerov_control.msg import Configuration


class SurgeControlNode(pid.PidNode):
    def __init__(self, name):
        super(SurgeControlNode, self).__init__(name=name)

        self.setpoint = 0.0

        self.configuration_sub = rospy.Subscriber("configuration",
                                                  Configuration,
                                                  self.on_configuration,
                                                  queue_size=1)

        self.thrust_pub = rospy.Publisher("thrust",
                                       Float64,
                                       queue_size=1)

    def on_configuration(self, msg):
        now = msg.header.stamp.to_sec()
        distance_to_object = msg.distance_to_object

        with self.data_lock:
            error = -(self.setpoint -  distance_to_object)
            u = self.update_controller(error=error, now=now)

        self.thrust_pub.publish(Float64(u))


def main():
    node = SurgeControlNode("surge_controller")
    node.run()


if __name__ == "__main__":
    main()