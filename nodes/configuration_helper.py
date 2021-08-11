#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node


class MyAdvancedNode(Node):
    def __init__(self, name):
        super(MyAdvancedNode, self).__init__(name=name)
        ...
if name == "__main__":
    my_node = MyAdvancedNode("my_advanced_node")
