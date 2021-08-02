#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool


import do_mpc
from vtest_model_ros import VTest_model
from VTest_MPC import VTest_mpc
from VTest_Simulator import VTest_simulator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

class vtest_main():
    def __init__(self):
        # Subscriber
        self.pose_sub = rospy.Subscriber('/lltv_pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscribeer('/lltv_vel', TwistStamped, self.twist_cb)

        #Publisher
        self.thrusterUp = rospy.Publisher('thruster_up', Bool, queue_size=1)
        self.thrusterDown = rospy.Publisher('thruster_down', Bool, queue_size=1)

        self.model = VTest_model()
        self.mpc = VTest_mpc(self.model)
        self.simulator = VTest_simulator(self.model)

        # Set the initial state for simulator
        self.x0 = np.array([0,0,17.0])
        self.simulator.x0 = self.x0
        self.mpc.x0 = self.x0

        self.total_pulse_time = 0
        self.fuel_rate = 0.018  #0.018 g/s

        # Parameters for bang-bang
        self.kappa = 0.1
        self.ud_last = np.array([[0],[0]])
        self.ud = np.array([[0],[0]])

        #self.mpc.set_initial_guess()
        def pose_cb(self, pose):
            self.x0[0] = pose.pose.position.z

        def twist_cb(self, twist):
            self.x0[1] = twist.twist.linear.z

if __name__ == '__main__':

    rospy.init_node('vtest_main_node', anonymous=False)
    rate = rospy.Rate(10)
    n = vtest_main()

    while not rospy.is_shutdown():
        u0 = n.mpc.make_step(n.x0)
        for cmd in range(2):
            if n.ud_last[cmd,0] == 1:
                if u0[cmd,0] >= (1-n.kappa)*1/2:
                    n.ud[cmd,0] = 1
                else:
                    n.ud[cmd,0] = 0
            elif n.ud_last[cmd,0] == 0:
                if u0[cmd,0] >= (1+n.kappa)*1/2:
                    n.ud[cmd,0] = 1
                else:
                    n.ud[cmd,0] = 0
            else:
                print('ERROR')
                quit()
        n.ud_last[0:2,0] = n.ud[0:2,0]


