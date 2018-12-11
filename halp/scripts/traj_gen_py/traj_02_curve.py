# This file mimics traj_02_curve.m

#
# generate data and verify the result of the C++ implementation
#

import numpy as np
from plan_car_trajectory import *


# =========================================================================
# plan
# =========================================================================
#dt = 0.05        # time step
#tf = 5           # final time
dt = 0.06        # time step
tf = 7.2           # final time
#tf = 0.6           # final time

tspan = np.arange(0,tf+dt,dt)

#s0 = np.array( [ [0], [0], [0], [0] ]  ) # initial state
#s1 = np.array( [ [50], [20], [0], [0] ]  ) # final state
s0 = np.array( [ [0], [0], [0], [0] ]  ) # initial state
s1 = np.array( [ [2], [1], [0], [0] ]  ) # final state
#s1 = np.array( [ [0.02], [0.01], [0], [0] ]  ) # final state

L = 0.68            # length of the car
kGains =  np.array( [[4],[4] ] )

(state,control) = plan_car_trajectory(L,s0,s1,kGains,tspan,dt)

output_trajectory(s0, state, control, 'py_trajectory_02.txt')


# .............................................................

out = np.vstack((state, control))
control_init = np.zeros((2, 1))
out_init = np.vstack((s0, control_init))
python_out = np.hstack((out_init, out)).transpose()


import matplotlib.pyplot as plt

def plotPose(matrix,format):
    state = matrix[:, 0:4]
    x = state[:, 0]
    y = state[:, 1]
    t = state[:, 2]
    ph = state[:, 3]
    control = matrix[:, 3:5]
    plt.plot(x,y,format)



matlab_out= np.loadtxt('./matlab/trajectory_02.txt')

plotPose(python_out,'xb')
plotPose(matlab_out,'.r')

err = (python_out - matlab_out)

e_x = err[:, 0]
e_y = err[:, 1]
e_a = err[:, 2]
e_p = err[:, 3]
e_v = err[:, 4]
e_w = err[:, 5]

print np.abs(e_x).max()
print np.abs(e_y).max()
print np.abs(e_a).max()
print np.abs(e_p).max()
print np.abs(e_v).max()
print np.abs(e_w).max()