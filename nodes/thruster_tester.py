#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float64
from bluerov_control.cfg import ThrusterTestConfig


class ThrusterTestNode(Node):
    def __init__(self, name):
        super(ThrusterTestNode, self).__init__(name=name)

        self.thrust = 0.0

        # change to desired thrust direction
        self.thrust_pub = rospy.Publisher("surge",
                                          Float64,
                                          queue_size=1)

        self.dynamic_reconfigure = Server(ThrusterTestConfig,
                                          self.on_reconfigure)

    def on_reconfigure(self, config, level):
        self.thrust = config.thruster_command
        return config

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.thrust_pub.publish(self.thrust)
            rate.sleep()


def main():
    node = ThrusterTestNode("thruster_tester")
    node.run()


if __name__ == "__main__":
    main()
