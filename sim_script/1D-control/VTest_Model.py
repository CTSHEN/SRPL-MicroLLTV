import numpy as np
import do_mpc
from casadi import *

def VTest_model():
    # Obtain an instance of the do-mpc model class
    # and select time discretization:
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # Introduce new states, inputs and other variables to the model, e.g.:
    r_1 = model.set_variable(var_type='_x', var_name='r_1', shape=(1,1))
    r_2 = model.set_variable(var_type='_x', var_name='r_2', shape=(1,1))
    m = model.set_variable(var_type='_x', var_name='m', shape=(1,1))

    Td = model.set_variable(var_type='_u', var_name='Td',shape=(1,1))
    Tu = model.set_variable(var_type='_u', var_name='Tu',shape=(1,1))

    position_setpoint = model.set_variable(var_type='_tvp',var_name='position_setpoint')
    velocity_setpoint = model.set_variable(var_type='_tvp',var_name='velocity_setpoint')
    
    # Some constant numbers
    m_2 = 8 # Mass of m2
    m_1s = 8.5 # Structure mass for m1
    m_1f = 0.5 # Initial fuel mass
    m_init = m_1s+m_1f+m_2
    fuelFlowRate = 0.018
    G = 9.81
    
    

    # Set right-hand-side of ODE for all introduced states (_x).
    # Names are inherited from the state definition.
    r2_next = 15*(Td-Tu)/m + (2*m_2/m -1)*G
    m_next = -fuelFlowRate*(Td+Tu)
    model.set_rhs('r_1', r_2)
    model.set_rhs('r_2', r2_next)
    model.set_rhs('m', m_next)

    # Setup model:
    model.setup()

    return model