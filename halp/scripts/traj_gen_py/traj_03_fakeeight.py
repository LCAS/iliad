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
tspan = np.arange(0,tf+dt,dt)


L = 0.68            # length of the car
kGains =  np.array( [[3],[3] ] )


s0 = np.array( [ [0], [0], [0], [0] ]  ) # initial state
s1 = np.array( [ [0], [-1.5], [-np.pi], [0] ]  ) # final state
(state1,control1) = plan_car_trajectory(L,s0,s1,kGains,tspan,dt)


# ................................................................

(P,dP,ddP,dddP,q,dq) = path_poly3(s0,s1,kGains, tspan)

x = P[0, :]
y = P[1, :]
dx = dP[0, :]
dy = dP[1, :]
ddx = ddP[0, :]
ddy = ddP[1, :]
dddx = dddP[0, :]
dddy = dddP[1, :]

# ...............................................................


s0 = state1[:,-1] # initial state
s1 = np.array( [ [0], [-3], [0], [0] ]  ) # final state
(state2,control2) = plan_car_trajectory(L,s0,s1,kGains,tspan,dt)

s0 = state2[:,-1] # initial state
s1 = np.array( [ [0], [-1.5], [np.pi], [0] ]  ) # final state
(state3,control3) = plan_car_trajectory(L,s0,s1,kGains,tspan,dt)

s0 = state3[:,-1] # initial state
s1 = np.array( [ [0], [0], [0], [0] ]  ) # final state
(state4,control4) = plan_car_trajectory(L,s0,s1,kGains,tspan,dt)

state = np.hstack((state1,state2,state3,state4))
control = np.hstack((control1,control2,control3,control4))

s0 = np.array( [ [0], [0], [0], [0] ]  )
output_trajectory(s0, state, control, 'py_trajectory_03.txt')


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



matlab_out= np.loadtxt('./matlab/trajectory_03.txt')

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