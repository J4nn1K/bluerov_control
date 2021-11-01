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

        self.object_ground_truth_pose_pub = rospy.Publisher("object_pose_ground_truth",
                                                            PoseStamped)

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            try:
                self.transformation = self.tfBuffer.lookup_transform(
                    self.tf_helper.get_base_link_ground_truth_id(), 'object_ground_truth', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            pose = PoseStamped()
            pose.pose.position = self.transformation.transform.translation
            pose.pose.orientation = self.transformation.transform.rotation
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.tf_helper.get_base_link_ground_truth_id()
            self.object_ground_truth_pose_pub.publish(pose)

            rate.sleep()


def main():
    node = GroundTruthHelperNode("ground_truth_helper")
    node.run()


if __name__ == "__main__":
    main()