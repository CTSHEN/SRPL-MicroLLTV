#!/usr/bin/env python3

"""
thruster_test.py

This script generate testing signals to topic
TD_cmd and TU_cmd to test the thruster control

"""
import rospy
from std_msgs.msg import Float32


"""
This function generates on-off signal sequence for up and downward
thrusters evert second
"""
def signal_gen():
    rospy.init_node("signal_gen", anonymous=True)

    TD = rospy.Publisher('TD_cmd', Float32, queue_size=1)
    TU = rospy.Publisher('TU_cmd', Float32, queue_size=1)

    TD.publish(0.0)
    TU.publish(0.0) #TU LOW
    print('TD 0, TU 0')
    rospy.sleep(1.0)
    TD.publish(1.0)  # TD HIGH
    print('TD 1, TU 0')
    rospy.sleep(1.0)
    TD.publish(0.0) # TD LOW
    print('TD 0, TU 0')
    rospy.sleep(1.0)
    TU.publish(1.0) # TU HGIH
    print('TD 0, TU 1')
    rospy.sleep(1.0)

    rospy.spin()

if __name__== "__main__":
    signal_gen()


