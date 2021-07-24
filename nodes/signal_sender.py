#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32

class InputGeneratorNode():
    def __init__(self,name):
        rospy.init_node(name)
        
        self.signal_pub = rospy.Publisher("signal", Float32, queue_size=1)

    def run(self):
        msg = Float32()
        while not rospy.is_shutdown():
            msg.data = 1.0
            self.signal_pub.publish(msg)
            
            rospy.sleep(8)
            
            msg.data = 0.0
            self.signal_pub.publish(msg)
            
            rospy.sleep(5)
            
            msg.data = -1.0
            self.signal_pub.publish(msg)
            
            rospy.sleep(8)
            
            msg.data = 0.0
            self.signal_pub.publish(msg)
            
            rospy.sleep(5)

def main():
    node = InputGeneratorNode("signal_generator")
    node.run()


if __name__ == "__main__":
    main()