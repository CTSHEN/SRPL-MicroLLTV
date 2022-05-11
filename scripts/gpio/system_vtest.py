#!/usr/bin/env python3

"""
system_vtest.py

This script generate testing signals to topic 
TD_cmd and TU_cmd to test the system on the fixed pully

"""
import rospy
from std_msgs.msg import Float32
import OneDThrusterCmdStamped.msg

def sys_vtest():
    rospy.init_node("system_vtest", anonymous=True)

    ThrusterCmd = OneDThrusterCmdStamped()

    TCmd = rospy.Publisher('Thruster_cmd', OneDThrusterCmdStamped, queue_size=1)

    #TD = rospy.Publisher('TD_cmd', Float32, queue_size=1)
    #TU = rospy.Publisher('TU_cmd', Float32, queue_size=1)
    while not rospy.is_shutdown():
        # Downward
        ThrusterCmd.TD_signal = True
        ThrusterCmd.TU_signal = False
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd)        
        rospy.sleep(2)

        # All off 
        ThrusterCmd.TD_signal = False
        ThrusterCmd.TU_signal = False
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd)       
        rospy.sleep(0.3)
        # Downward 
        ThrusterCmd.TD_signal = True
        ThrusterCmd.TU_signal = False
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd) 
        rospy.sleep(2)

        # All off 
        ThrusterCmd.TD_signal = False
        ThrusterCmd.TU_signal = False
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd)         
        rospy.sleep(0.3)

        # Upward 
        ThrusterCmd.TD_signal = False
        ThrusterCmd.TU_signal = True
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd)        
        rospy.sleep(1.0)

        # Downward 
        ThrusterCmd.TD_signal = True
        ThrusterCmd.TU_signal = False
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd) 
        rospy.sleep(1.0)
        
        # All off 10s
        ThrusterCmd.TD_signal = False
        ThrusterCmd.TU_signal = False
        ThrusterCmd.header.stamp = rospy.get_rostime()
        TCmd.publish(ThrusterCmd) 
        rospy.sleep(10.0)

if __name__== "__main__":
    sys_vtest()
