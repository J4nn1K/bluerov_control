#!/usr/bin/env python

import threading
import rospy
import math
from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
from bluerov_control.msg import Configuration


class ConfigurationHelperNode(Node):
    def __init__(self, name):
        super(ConfigurationHelperNode, self).__init__(name=name)

        self.data_lock = threading.RLock()

        self.object_pose_sub = rospy.Subscriber("object_pose",
                                                PoseStamped,
                                                self.on_object_pose,
                                                queue_size=1)

        self.configuration_pub = rospy.Publisher("configuration",
                                                 Configuration,
                                                 queue_size=1)

    def on_object_pose(self, msg):
        with self.data_lock:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            angle_to_object = math.atan(y/x)
            distance_to_object = math.sqrt(x**2 + y**2) - 0.38 # Offset: base_link to gripper_link
            
            message = Configuration(angle_to_object=angle_to_object,
                                    distance_to_object=distance_to_object)
            message.header.stamp = msg.header.stamp

            self.configuration_pub.publish(message)


def main():
    node = ConfigurationHelperNode("configuration_helper")
    node.run()

if __name__ == "__main__":
    main()
    