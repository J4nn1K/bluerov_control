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

        self.use_localization = self.get_param("~use_localization",
                                               default=False)

        if self.use_localization:
            self.pose_from_object_sub = rospy.Subscriber("pose_from_object",
                                                    PoseStamped,
                                                    self.on_pose,
                                                    queue_size=1)

        else:
            self.pose_from_object_ground_truth_sub = rospy.Subscriber("pose_from_object_ground_truth",
                                                                 PoseStamped,
                                                                 self.on_pose,
                                                                 queue_size=1)

        self.configuration_pub = rospy.Publisher("configuration",
                                                 Configuration,
                                                 queue_size=1)

    def on_pose(self, msg):
        with self.data_lock:
            # all data referenced in object frame
            x = msg.pose.position.x
            y = msg.pose.position.y

            r = math.sqrt(x**2 + y**2)

            orientation = msg.pose.orientation
            orientation_euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            body_orientation_z = orientation_euler[2]
            # convert orientation of object to positive angles
            if(body_orientation_z < 0):
                body_orientation_z += 2*math.pi

            # calculate angle from object x-axis to line of sight
            if y >= 0:
                object_to_LOS = math.acos(x/r)
            elif y < 0:
                object_to_LOS = 2*math.pi - math.acos(x/r)

            # calculate ange from body x-axis to line of sight
            body_to_LOS = math.pi - body_orientation_z + object_to_LOS
            # cut to range [0, 2*pi]
            if(body_to_LOS < 0):
                body_to_LOS += 2*math.pi
            if(body_to_LOS > 2*math.pi):
                body_to_LOS -= 2*math.pi

            # transform to range [-pi, pi]
            if(object_to_LOS > math.pi):
                object_to_LOS -= 2*math.pi
            if(body_to_LOS > math.pi):
                body_to_LOS -= 2*math.pi

            distance = r

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
