#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
# from bluerov_control import pid
from control import pid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class DepthControlNode(pid.PidNode):
    def __init__(self, name):
        super(DepthControlNode, self).__init__(name=name)

        # bei ground truth: 6 1 1 (P I D)
        # TODO zweite PID Regler Form in Control Package schreiben
        # .cfg files in bluerovcontrol aber regler aus Control aufrufen (in Launch files anpassen)

        self.setpoint = 0.0

        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

        self.pressure_depth_pub = rospy.Publisher("depth_estimate",
                                                  Float64,
                                                  queue_size=1)

        self.depth_setpoint_sub = rospy.Subscriber("depth_setpoint",
                                                   Float64,
                                                   self.on_depth_setpoint,
                                                   queue_size=1)

#        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose",
#                                               PoseStamped,
#                                               self.on_local_pose,
#                                               queue_size=1)

        self.pressure_sub = rospy.Subscriber("/pressure",
                                             FluidPressure,
                                             self.on_pressure,
                                             queue_size=1)

#        self.ground_truth_sub = rospy.Subscriber("ground_truth/state",
#                                                Odometry,
#                                                self.on_ground_truth,
#                                                queue_size=1)

    def on_depth_setpoint(self, msg):
        self.setpoint = msg.data

    def on_local_pose(self, msg):
        z = msg.pose.position.z
        now = msg.header.stamp.to_sec()

        with self.data_lock:
            error = self.setpoint - z
            u = self.update_controller(error=error, now=now)

        self.vertical_thrust_pub.publish(Float64(u))

    def on_pressure(self, msg):
        p_0 = 101325  # [Pa]
        rho = 1000  # [kg/m]
        g = 9.81  # [m/s]

        z = -(msg.fluid_pressure - p_0)/(rho*g)

        now = msg.header.stamp.to_sec()

        with self.data_lock:
            error = self.setpoint - z
            u = self.update_controller(error=error, now=now)
        # rospy.loginfo(z)
        self.pressure_depth_pub.publish(z)
        self.vertical_thrust_pub.publish(Float64(u))

    def on_ground_truth(self, msg):
        z = msg.pose.pose.position.z
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
