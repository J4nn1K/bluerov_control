#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float64

from bluerov_control.cfg import DepthControlConfig


class DepthSetpointNode(Node):
    def __init__(self, name):
        super(DepthSetpointNode, self).__init__(name=name)
        
        self.setpoint = -0.5
        
        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            Float64,
                                            queue_size=1)

        self.dynamic_reconfigure = Server(DepthControlConfig,
                                          self.on_reconfigure)

    def on_reconfigure(self, config, level):
        self.setpoint = config.depth_setpoint
        return config

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.setpoint_pub.publish(self.setpoint)
            rate.sleep()
    
def main():
    node = DepthSetpointNode("DepthSetpoint")
    node.run()


if __name__ == "__main__":
    main()
