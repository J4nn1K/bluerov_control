#!/usr/bin/env python
import rospy
from bluerov_sim.msg import ActuatorCommands
from mavros_msgs.msg import MotorSetpoint


class MixerNode():
    def __init__(self, name):
        rospy.init_node(name)
        self.thrusters = self.init_mixing()
        self.motor_setpoint_pub = rospy.Publisher(
            "mavros/setpoint_motor/setpoint", MotorSetpoint, queue_size=1)
        rospy.Subscriber("~actuator_commands",
                         ActuatorCommands,
                         self.actuator_callback,
                         queue_size=1)

    def actuator_callback(self, msg):
        output = MotorSetpoint()
        output.header.stamp = rospy.Time.now()
        for i, thruster in enumerate(self.thrusters):
            output.setpoint[i] = (
                thruster["roll"] * msg.roll + thruster["pitch"] * msg.pitch +
                thruster["yaw"] * msg.yaw + thruster["thrust"] * msg.thrust +
                thruster["lateral_thrust"] * msg.lateral_thrust +
                thruster["vertical_thrust"] * msg.vertical_thrust)
        self.motor_setpoint_pub.publish(output)

    def init_mixing(self):
        thruster = [None] * 8
        # roll, pitch, yaw, thrust, lateral thrust, vertical thrust
        thruster[0] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=1.0,
                           thrust=1.0,
                           lateral_thrust=1.0,
                           vertical_thrust=0.0)
        thruster[1] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=-1.0,
                           thrust=1.0,
                           lateral_thrust=-1.0,
                           vertical_thrust=0.0)
        thruster[2] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=1.0,
                           thrust=1.0,
                           lateral_thrust=-1.0,
                           vertical_thrust=0.0)
        thruster[3] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=-1.0,
                           thrust=1.0,
                           lateral_thrust=1.0,
                           vertical_thrust=0.0)
        thruster[4] = dict(roll=-1.0,
                           pitch=-1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=1.0)
        thruster[5] = dict(roll=-1.0,
                           pitch=1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=-1.0)
        thruster[6] = dict(roll=1.0,
                           pitch=-1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=-1.0)
        thruster[7] = dict(roll=1.0,
                           pitch=1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=1.0)

        return thruster

def main():
    node = MixerNode("mixer")
    rospy.spin()


if __name__ == "__main__":
    main()
