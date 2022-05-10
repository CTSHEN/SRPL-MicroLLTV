#!/usr/bin/env python3

"""
system_vtest.py

This script generate testing signals to topic 
TD_cmd and TU_cmd to test the system on the fixed pully

"""
import rospy
from std_msgs.msg import Float32

def sys_vtest():
    rospy.init_node("system_vtest", anonymous=True)

    TD = rospy.Publisher('TD_cmd', Float32, queue_size=1)
    TU = rospy.Publisher('TU_cmd', Float32, queue_size=1)
    while not rospy.is_shutdown():
        # Downward 0.3s
        TD.publish(1.0)
        TU.publish(0.0)
        rospy.sleep(2)
        # All off 0.1s
        TD.publish(0.0)
        TU.publish(0.0)
        rospy.sleep(0.3)
        # Downward 0.3s
        TD.publish(1.0)
        TU.publish(0.0)
        rospy.sleep(2)
        # All off 0.1s
        TD.publish(0.0)
        TU.publish(0.0)
        rospy.sleep(0.3)
        # Upward 0.3s
        TD.publish(0.0)
        TU.publish(1.0)
        rospy.sleep(0.5)
        # Downward 0.3s
        TD.publish(1.0)
        TU.publish(0.0)
        rospy.sleep(1.0)
        # All off 10s
        TD.publish(0.0)
        TU.publish(0.0)
        rospy.sleep(10.0)

if __name__== "__main__":
    sys_vtest()
