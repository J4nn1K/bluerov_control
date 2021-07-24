#!/usr/bin/env python

import rospy
from bluerov_sim.msg import ActuatorCommands
from std_msgs.msg import Float32, Float64

class InputGeneratorNode():
    def __init__(self,name):
        rospy.init_node(name)
        self.value = 0.0
        
        self.actuator_pub = rospy.Publisher("bluerov/lateral_thrust",
                                            Float64,
                                            queue_size=1)

        self.signal_sub = rospy.Subscriber("signal", Float32, self.on_signal)                   
       
    def on_signal(self, msg):
        self.value = msg.data
    
    def run(self):
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            msg = Float64()
            msg.data = self.value
            self.actuator_pub.publish(msg)
            rate.sleep()

def main():
    node = InputGeneratorNode("input_generator")
    node.run()


if __name__ == "__main__":
    main()