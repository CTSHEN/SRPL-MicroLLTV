import numpy as np
import do_mpc
from casadi import *

def VTest_mpc(model):
    # Obtain an instance of the do-mpc MPC class
    # and initiate it with the model:
    mpc = do_mpc.controller.MPC(model)

    # Set parameters:
    setup_mpc = {
        'n_horizon': 20,
        'n_robust': 1,
        't_step': 0.3,
        'store_full_solution': True
    }
    mpc.set_param(**setup_mpc)
    mpc.set_param(store_full_solution=True)

    _x = model.x
    _u = model.u
    _tvp = model.tvp
    # Reference setpoint
    #r1_goal = 1
    #r2_goal = 0

    # Configure objective function:
    mterm = (_x['r_1'] - _tvp['position_setpoint'])**2 + 0.1*(_x['r_2'] - _tvp['velocity_setpoint'])**2   # Setpoint tracking
    lterm = 0.1*(_u['Td']**2 + _u['Tu']**2) + (_x['r_1'] - _tvp['position_setpoint'])**2 + 0.1*(_x['r_2'] - _tvp['velocity_setpoint'])**2    

    mpc.set_objective(mterm=mterm, lterm=lterm)
    

    # State and input bounds:
    mpc.bounds['lower', '_x', 'r_1'] = 0
    mpc.bounds['upper', '_x', 'r_1'] = 3.0
    mpc.bounds['lower', '_x', 'm']= 16.5
    mpc.bounds['upper', '_x', 'm']= 17.0

    mpc.bounds['lower', '_u', 'Td'] = 0
    mpc.bounds['upper', '_u', 'Td'] = 1
    mpc.bounds['lower', '_u', 'Tu'] = 0
    mpc.bounds['upper', '_u', 'Td'] = 1

    tvp_template = mpc.get_tvp_template()
    def tvp_fun(t_now):
        if t_now < 15:
            tvp_template['_tvp',:,'position_setpoint'] = 1
            tvp_template['_tvp',:,'velocity_setpoint'] = 0
        else:
            tvp_template['_tvp',:,'position_setpoint'] = 0
            tvp_template['_tvp',:,'velocity_setpoint'] = 0
        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()

    return mpc