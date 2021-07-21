#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool

def arm_vehicle():
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while not arm(True).success:
            rospy.logwarn_throttle(1, "Could not arm vehicle. Keep trying.")
        rospy.loginfo("Armed successfully.")

def disarm_vehicle():
        rospy.wait_for_service("mavros/cmd/arming")
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arm(False)
        rospy.loginfo("Disarmed successfully.")

def main():
    rospy.init_node('armed')
    arm_vehicle()

    #rospy.on_shutdown(disarm_vehicle())

if __name__ == "__main__":
    main()