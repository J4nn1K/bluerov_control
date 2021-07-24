#!/usr/bin/env python

import rospy
from bluerov_control import pid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class DepthControlNode(pid.PidNode):
    def __init__(self, name):
        super(DepthControlNode, self).__init__(name=name)

        self.setpoint = -0.6

        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose",
                                               PoseStamped,
                                               self.on_local_pose,
                                               queue_size=1)

    def on_local_pose(self, msg):
        z = msg.pose.position.z
        now = msg.header.stamp.to_sec()

        with self.data_lock:
            error = self.setpoint - z
            u = self.update_controller(error=error, now=now)

        self.vertical_thrust_pub.publish(Float64(u))


def main():
    node = DepthControlNode("depth_controller")
    node.run()


if __name__ == "__main__":
    main()
