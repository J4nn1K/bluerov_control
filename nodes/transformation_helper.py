#!/usr/bin/env python

import rospy
import tf2_ros
from hippocampus_common.node import Node
from hippocampus_common.tf_helper import TfHelper
from geometry_msgs.msg import PoseStamped


class TransformationHelperNode(Node):
    def __init__(self, name):
        super(TransformationHelperNode, self).__init__(name=name)

        self.tf_helper = TfHelper("bluerov")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pose_from_object_pub = rospy.Publisher("pose_from_object", PoseStamped)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            try:
                self.transformation = self.tfBuffer.lookup_transform(
                    'object_frame', self.tf_helper.get_base_link_id(), rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            pose = PoseStamped()
            pose.pose.position = self.transformation.transform.translation
            pose.pose.orientation = self.transformation.transform.rotation
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "object_frame"
            self.pose_from_object_pub.publish(pose)

            rate.sleep()


def main():
    node = TransformationHelperNode("transformation_helper")
    node.run()


if __name__ == "__main__":
    main()