#!/usr/bin/env python
import threading
import rospy
from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure

class DifferentiatorNode(Node):
    def __init__(self, name):
        super(DifferentiatorNode, self).__init__(name=name)
        
        self.data_lock = threading.RLock()
        
        self.t_last = rospy.get_time()
        self.last_value = 0.0

        self.derivative = 0.0
        

        self.pose_sub = rospy.Subscriber("bluerov/mavros/vision_pose/pose",
                                             PoseStamped,
                                             self.on_pose,
                                             queue_size=1)
                                        
    def on_pose(self, msg):
        
        now = msg.header.stamp.to_sec()
        value = msg.pose.position.x

        #self.update_dt(now)
        self.update_derivative(value, self.update_dt(now))

    def update_dt(self, now):
        dt = now - self.t_last
        self.t_last = now
        
        #rospy.loginfo(dt)
        if dt>0:
            return dt
        else:
            return 0.01
        

    def update_derivative(self, value, dt):
        self.derivative = (value - self.last_value) / dt

        rospy.loginfo(self.derivative)    

def main():
    node = DifferentiatorNode("differentiator")
    node.run()

if __name__ == "__main__":
    main()