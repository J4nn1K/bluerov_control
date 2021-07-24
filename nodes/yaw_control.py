#!/usr/bin/env python

import rospy
from bluerov_control import pid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class YawControlNode(pid.PidNode):
    def __init__(self, name):
        super(YawControlNode, self).__init__(name=name)

        self.setpoint = 0.0

        self.yaw_pub = rospy.Publisher("yaw",
                                       Float64,
                                       queue_size=1)

        self.local_pose_sub = rospy.Subscriber("object_pose",
                                               PoseStamped,
                                               self.on_local_pose,
                                               queue_size=1)

    def on_local_pose(self, msg):
        rospy.loginfo(msg)
        y = msg.pose.position.y
        now = msg.header.stamp.to_sec()

        with self.data_lock:
            error = -(self.setpoint - y)
            u = self.update_controller(error=error, now=now)

        self.yaw_pub.publish(Float64(u))


def main():
    node = YawControlNode("yaw_controller")
    node.run()


if __name__ == "__main__":
    main()
