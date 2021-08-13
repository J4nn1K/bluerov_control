#!/usr/bin/env python

import threading
import rospy
import math
from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from bluerov_control.msg import Configuration


class ConfigurationHelperNode(Node):
    def __init__(self, name):
        super(ConfigurationHelperNode, self).__init__(name=name)
        self.data_lock = threading.RLock()

        self.use_ground_truth = self.get_param("~use_ground_truth",
                                               default=True)

        if self.use_ground_truth:
            self.object_pose_ground_truth_sub = rospy.Subscriber("object_pose_ground_truth",
                                                                 PoseStamped,
                                                                 self.on_object_pose,
                                                                 queue_size=1)
        else:
            self.object_pose_sub = rospy.Subscriber("object_pose",
                                                    PoseStamped,
                                                    self.on_object_pose,
                                                    queue_size=1)

        self.configuration_pub = rospy.Publisher("configuration",
                                                 Configuration,
                                                 queue_size=1)

    def on_object_pose(self, msg):
        with self.data_lock:
            # all data referenced in body frame
            x = msg.pose.position.x
            y = msg.pose.position.y
            orientation = msg.pose.orientation
            orientation_euler = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])
            object_orientation_z = orientation_euler[2]
            # calculate only positive angles:
            if(object_orientation_z < 0):
                object_orientation_z += 2*math.pi
            # rospy.loginfo(object_orientation_z*(180/math.pi))

            if x > 0 and y >= 0:
                body_to_LOS = math.atan(y/x)
            elif x > 0 and y < 0:
                body_to_LOS = math.atan(y/x) + 2*math.pi
            elif x < 0:
                body_to_LOS = math.atan(y/x) + math.pi
            elif x == 0 and y > 0:
                body_to_LOS = math.pi/2
            elif x == 0 and y < 0:
                body_to_LOS = 3*math.pi/2

            object_to_LOS = math.pi - object_orientation_z + body_to_LOS

            if(object_to_LOS < 0):
                object_to_LOS += 2*math.pi
            if(object_to_LOS > 2*math.pi):
                object_to_LOS -= 2*math.pi
            # rospy.loginfo((object_to_LOS/math.pi)*180)

            # transform in range [-pi, pi]
            if(body_to_LOS > math.pi):
                body_to_LOS -= 2*math.pi
            if(object_to_LOS > math.pi):
                object_to_LOS -= 2*math.pi

            # Offset: base_link to gripper_link
            distance = math.sqrt(x**2 + y**2) - 0.38

            message = Configuration(distance=distance,
                                    body_to_LOS=body_to_LOS,
                                    body_to_LOS_deg=body_to_LOS*(180/math.pi),
                                    object_to_LOS=object_to_LOS,
                                    object_to_LOS_deg=object_to_LOS*(180/math.pi))
            message.header.stamp = msg.header.stamp

            self.configuration_pub.publish(message)


def main():
    node = ConfigurationHelperNode("configuration_helper")
    node.run()


if __name__ == "__main__":
    main()
