#!/usr/bin/env python3

"""
purge_vlave_control.py

This script pass the input signal from DAQ
to control the purge valve.

Input: 
    DAQ input:
        GPIO 26 PIN 37
OUTPUT:
    Purge valve:
        GPIO 21 PIN 40
"""
import rospy
from gpiozero import LED,InputDevice #DigitalInputDevice

# Define GPIO
DAQ_in = InputDevice(26) #DigitalInputDevice(26)
valve_out = LED(21)

def signal_pass_through():
    rospy.init_node('purge_valve_control_node', anonymous=True)
    rate = rospy.Rate(2) # check every 0.5 sec
    valve_out.off()

    while not rospy.is_shutdown():
        if DAQ_in.is_active: #DAQ_in.when_activated:
            valve_out.on()
        else:
            valve_out.off()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        signal_pass_through()
    except rospy.ROSInterruptException:
        pass
