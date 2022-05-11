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
#from std_msgs.msg import Float32
import OneDThrusterCmdStamped.msg
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

def Thruster_Cb(msg):
    assert msg.TD_signal == True or msg.TD_signal == False, 'TD command error'
    assert msg.TU_signal == True or msg.TU_signal == False, 'TU commnad error'

    if msg.TD_signal == False and msg.TU_signal == False:
        TD.off()
        TU.off()
    elif msg.TD_signal == True and msg.TU_signal == False:
        TD.on()
        TU.off()
    elif msg.TD_signal == False and msg.TU_signal == True:
        TD.off()
        TU.on()
    elif msg.TD_signal == True and msg.TU_signal == True:
        TD.on()
        TU.on()
    else:
        print(msg.TD_signal)
        print(msg.TU_signal)
        exit()

    

def thruster_startup():
    TD.on()
    TU.on()
    rospy.sleep(5.0) # wait 5 seconds

    TD.off()
    TU.off()

def gpio_control():
    rospy.init_node('thruster_control_gpio', anonymous=True)

    rospy.Subscriber("Thruster_cmd", OneDThrusterCmdStamped, Thruster_Cb)

    rospy.spin()

if __name__ == '__main__':
    try:
        thruster_startup()

        gpio_control()
    except rospy.ROSInterruptException:
        pass
