import numpy as np
import do_mpc
from casadi import *


def VTest_simulator(model):
    # Obtain an instance of the do-mpc simulator class
    # and initiate it with the model:
    simulator = do_mpc.simulator.Simulator(model)

    # Set parameter(s):
    simulator.set_param(t_step = 0.3)

    # Optional: Set function for parameters and time-varying parameters.
    tvp_template = simulator.get_tvp_template()
    def tvp_fun(t_now):
        return tvp_template
    simulator.set_tvp_fun(tvp_fun)

    # Setup simulator:
    simulator.setup()

    return simulator