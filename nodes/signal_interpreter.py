#!/usr/bin/env python

import rospy
from bluerov_sim.msg import ActuatorCommands
from std_msgs.msg import Float32

class InputGeneratorNode():
    def __init__(self,name):
        rospy.init_node(name)
        self.thrust = 0.0
        
        self.actuator_pub = rospy.Publisher("bluerov/mixer/actuator_commands",
                                            ActuatorCommands,
                                            queue_size=1)

        self.signal_sub = rospy.Subscriber("signal", Float32, self.on_signal)                   
       
    def on_signal(self, msg):
        self.thrust = msg.data
    
    def run(self):
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            msg = ActuatorCommands()
            msg.header.stamp = rospy.Time.now()
            msg.thrust = self.thrust
            self.actuator_pub.publish(msg)
            rate.sleep()

def main():
    node = InputGeneratorNode("input_generator")
    node.run()


if __name__ == "__main__":
    main()