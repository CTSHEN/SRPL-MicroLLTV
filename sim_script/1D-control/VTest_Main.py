import do_mpc
from VTest_MPC import VTest_mpc
from VTest_Model import VTest_model
from VTest_MPC import VTest_mpc
from VTest_Simulator import VTest_simulator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# Constants
N = 100

model = VTest_model()
mpc = VTest_mpc(model)
simulator = VTest_simulator(model)

# Set the initial state of simulator
x0 = np.array([0, 0, 17.0])
simulator.x0 = x0
mpc.x0 = x0

mpc.set_initial_guess()

# Customizing Matplotlib
mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True

# Setup graphics
mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

fig, ax = plt.subplots(5, sharex=True, figsize=(16,10))
fig.align_ylabels()
plt.ion()

for g in [sim_graphics, mpc_graphics]:
    # Plot r1, r2 and m on the first axis
    g.add_line(var_type='_x', var_name='r_1', axis=ax[0])
    g.add_line(var_type='_x', var_name='r_2', axis=ax[0])
    g.add_line(var_type='_tvp', var_name='position_setpoint', axis=ax[0], linestyle='--')
    g.add_line(var_type='_x', var_name='m', axis=ax[1])

    # Plot control inputs
    g.add_line(var_type='_u', var_name='Td', axis=ax[2])
    g.add_line(var_type='_u', var_name='Tu', axis=ax[2])

ax[0].set_title('Simulation Result')
ax[0].set_ylabel('meter')
ax[0].legend(mpc_graphics.result_lines['_x','r_1']
     + mpc_graphics.result_lines['_x','r_2']+ mpc_graphics.result_lines['_tvp','position_setpoint']
     , ['position', 'velocity','position_setpoint'], fontsize=10)
ax[0].set_ylim([-1,1.5])
ax[0].set_xlim([0,31])

ax[1].set_title('Total Mass', fontsize =12)
ax[1].set_ylabel('kg')
ax[1].legend(mpc_graphics.result_lines['_x','m'],['total mass'], fontsize=10)
ax[1].set_ylim([16.5,17.1])
ax[1].set_xlim([0,31])

ax[2].set_title('control_signal', fontsize =12)
#ax[2].set_xlabel('time [s]')
ax[2].legend(mpc_graphics.result_lines['_u','Td'] 
    + mpc_graphics.result_lines['_u','Tu'],['Thrust Down','Thrust Up'], fontsize=10)
ax[2].set_ylim([0,1])
ax[2].set_xlim([0,31])


_x = model.x
# Running Simulator
#while mpc._x['m'] >= 6.5: #(model.m_2 + model.m_1s):

# some parameters for bang-bang
kappa = 0.1
ud_last = np.array([[0],[0]])
ud = np.zeros((2,N))

for k in range(N):
#u0 = mpc.make_step(x0)
#while mpc.data['_x','m',0] >= 6.5:
    u0 = mpc.make_step(x0)
    for cmd in range(2):
        if ud_last[cmd,0] == 1:
            if u0[cmd,0] >= (1-kappa)*1/2:
                ud[cmd,k] = 1
            else:
                ud[cmd,k] = 0
        elif ud_last[cmd,0] == 0:
            if u0[cmd,0] >= (1+kappa)*1/2:
                ud[cmd,k] = 1
            else:
                ud[cmd,k] = 0
        else:
            print('ERROR')
            quit()
        
    ud_last = ud[0:2,k:k+1]
    
    x0 = simulator.make_step(ud[0:2,k:k+1])
    #u0 = mpc.make_step(x0)

    ax[3].set_title('outputDown', fontsize=10)
    ax[3].plot(mpc.data['_time'],ud[0:1,0:k+1].transpose(),drawstyle='steps-post',color='orange')
    ax[4].set_title('outputUp', fontsize=10)
    ax[4].set_xlabel('time [s]')
    ax[4].plot(mpc.data['_time'],ud[1:2,0:k+1].transpose(),drawstyle='steps-post',color='blue')

    mpc_graphics.plot_results()
    #sim_graphics.plot_results()
    #mpc_graphics.reset_axes()
    #sim_graphics.reset_axes()
    plt.show()
    plt.pause(0.01)

input('Press ENTER to exit.')

# store result
#do_mpc.data.save_results([mpc, simulator], 'VTest')
