#!/usr/bin/env python3

""" 
thruster_control_gpio.py

This script controls both thrusters via GPIO
and begin with the Thruster startup procedure.
OUTPUT
    Downward thruster:
        GPIO 16 PIN 36
    Upward thruster:
        GPIO 20 PIN 38

Common GND: 
PIN 34

"""
import rospy
from std_msgs.msg import Float32
# Use LED API to control the on-off device
from gpiozero import LED

# Define GPIO
TD = LED(16)
TU = LED(20)

"""
 Callback functions

 These functions control the level of the GPIOs w.r.t.
 the thruster commands.
"""

def TD_Cb(msg):
    assert msg.data == 0 or msg.data == 1, 'TD command error'
    
    if msg.data == 0:
        TD.off()
    elif msg.data == 1:
        TD.on()
    else:
        print(msg.data)
        exit()

def TU_Cb(msg):
    assert msg.data == 0 or msg.data == 1, 'TD command error'
    
    if msg.data == 0:
        TU.off()
    elif msg.data == 1:
        TU.on()
    else:
        print(msg.data)
        exit()

    

def thruster_startup():
    TD.on()
    TU.on()
    rospy.sleep(5.0) # wait 5 seconds

    TD.off()
    TU.off()

def gpio_control():
    rospy.init_node('thruster_control_gpio', anonymous=True)

    rospy.Subscriber("TD_cmd", Float32, TD_Cb)
    rospy.Subscriber("TU_cmd", Float32, TU_Cb)

    rospy.spin()

if __name__ == '__main__':
    try:
        #thruster_startup()

        gpio_control()
    except rospy.ROSInterruptException:
        pass
