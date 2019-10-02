"""

# Velocity constraints for human aware local avoidance

## Goal.
We want to obtain translational and rotational speed constraints for the robot in presence of a human, compatible with some qualitative constraints.

## Inputs.
- Human state vector: position (xh,yh,hh), and speed (vh,wh)
- Robot state vector: position (xr,yr,hr), and speed (vr,wr)
- Set of forbidden qtc states: i.e. {(----), (-0--), ...}

## Output.
- Range of acceptable speeds for robot (vr_min,vr_max) (wr_min, wr_max)

## Approaches:
- Brute force: we generate a 2D grid of speeds, from them corresponding qtc states, from them we identify valid/invalid space and finally get biggest bounding box

"""

# ****************************************************************************************************

from __future__ import print_function, division
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib.qsrlib import QSR_QTC_BC_Simplified

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ****************************************************************************************************

def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
    print(which_qsr, "request was made at ", str(qsrlib_response_message.req_made_at)
          + " and received at " + str(qsrlib_response_message.req_received_at)
          + " and finished at " + str(qsrlib_response_message.req_finished_at))
    print("---")
    print("Response is:")
    for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
        foo = str(t) + ": "
        for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
                        qsrlib_response_message.qsrs.trace[t].qsrs.values()):
            foo += str(k) + ":" + str(v.qsr) + "; "
        print(foo)

# ****************************************************************************************************

# https://codegolf.stackexchange.com/questions/79498/find-a-maximal-rectangle-of-1s
def maximalRectangle(M):
    n,m = M.shape

    # Count number of 1 bits in each column
    cumul_height = np.zeros((n,m))

    # first row
    cumul_height[0, :] = M[0, :]

    # rest of matrix
    for i in range(1, n):
        for j in range(0, m):
            if M[i,j]==1:
                cumul_height[i,j] = cumul_height[i-1,j] + 1
            else:
                cumul_height[i,j] = 0

    max_area=0
    inc_i=0
    inc_j=0
    max_i = 0
    min_i = 0
    min_j = 0
    max_j = 0

    # for each row we
    for i in range(0, n):
        # We compute all contiguous sublists of each row
        row= cumul_height[i, :]
        for width in range(1, len(row) + 1):
            for offset in range(0, m - width + 1):
                slice = row[0 + offset:offset + width]
                slice_area = width * np.min(slice)
                if slice_area>max_area:
                    max_area=slice_area
                    inc_i=int(np.min(slice))
                    inc_j=width # len(slice)
                    max_i=i
                    min_i=max_i-inc_i+1
                    min_j = offset
                    max_j=min_j+inc_j-1

    # print("xxxxxxxxxxxx")
    # print("Dimmensions (w x h) (%d x %d)" % (inc_i,inc_j))
    # print("xxxxxxxxxxxx")
    # print("M:")
    # print(M)
    # print("xxxxxxxxxxxx")
    # print("Start at: %d,%d" % (min_i,min_j))
    # print("End at:   %d,%d" % (max_i,max_j))
    # print("xxxxxxxxxxxx")
    return max_area, min_i,min_j,max_i,max_j

# a = [
#  [[0]],
#
#  [[1]],
#
#  [[0,0],
#   [0,0]],
#
#  [[0,1],
#   [1,0]],
#
#  [[0,1],
#   [1,1]],
#
#  [[1,1,1],
#   [0,1,0],
#   [1,1,1]],
#
#  [[1,0,1],
#   [0,1,1],
#   [1,1,1]],
#
#  [[0,1,1,1],
#   [1,1,1,0],
#   [1,1,0,0]],
#
#  [[1,1,1,1,1,1,1],
#   [1,1,1,0,1,1,1],
#   [1,0,1,1,1,0,1]],
#
#  [[1,1,1,0,1,1,0,0,0],
#   [1,1,0,1,1,1,1,0,0],
#   [0,0,1,1,1,1,1,1,0],
#   [0,1,1,1,1,1,1,1,1],
#   [0,0,1,1,1,1,1,1,0],
#   [0,0,0,1,1,1,1,0,0],
#   [0,0,0,0,1,1,0,0,0]],
#
#  [[0,0,0,1,1,0,0,0,0],
#   [1,1,0,1,1,0,0,1,0],
#   [1,1,0,1,1,1,1,1,0],
#   [1,1,0,0,1,1,1,0,0],
#   [0,1,0,0,1,1,1,1,1],
#   [1,1,1,1,1,1,1,1,1],
#   [1,1,1,1,0,1,1,1,0]]
# ]
#
#
# for mi in a:
#     maximalRectangle(np.array(mi))


# ****************************************************************************************************

def isValidState(xh0,yh0,xh1,yh1,xr0,yr0,xr1,yr1,td, which_qsr, dynammic_args, forbidden_states):
    ans = 1

    human_os = [Object_State(name="human_os", timestamp=0, x=xh0, y=yh0),
                Object_State(name="human_os", timestamp=td, x=xh1, y=yh1)
                ]

    robot_os = [Object_State(name="robot_os", timestamp=0, x=xr0, y=yr0),
                Object_State(name="robot_os", timestamp=td, x=xr1, y=yr1)
                ]

    # make some input data
    world = World_Trace()

    world.add_object_state_series(human_os)
    world.add_object_state_series(robot_os)

    # make a QSRlib request message
    qsrlib_request_message = QSRlib_Request_Message(which_qsr, world, dynammic_args)

    # request your QSRs
    qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)

    # should have 1 timestamp
    t = qsrlib_response_message.qsrs.get_sorted_timestamps()
    if len(t)!=1:
        print("something is wrong!!!!!!!!!!!!! with t")
        sys.exit(1)
    t = t[0]

    # should have one value: human to robot
    v = qsrlib_response_message.qsrs.trace[t].qsrs.values()
    if len(v)!=1:
        print("something is wrong!!!!!!!!!!!!! with v")
        sys.exit(1)
    v = v[0]
    state = v.qsr.values()
    if len(state)!=1:
        print("something is wrong!!!!!!!!!!!!! with state")
        sys.exit(1)
    state=state[0]

    if state in forbidden_states:
        ans = 0
        #print(state)
    return (ans,state)

# ****************************************************************************************************

# arguments
which_qsr = 'qtcbcs'
state_space = QSR_QTC_BC_Simplified().all_possible_relations
states_len = [ len(si.replace(',','')) for si in state_space]

dynammic_args = {"qtcbcs": {"no_collapse": True, "quantisation_factor":0.01, "validate":False, "qsrs_for":[("human_os","robot_os")] }}

# create a QSRlib object if there isn't one already
qsrlib = QSRlib()
qsr_types = qsrlib.qsrs_registry.keys()

# Print available qsr models
# qsrlib._QSRlib__print_qsrs_available()

# Print available states
# qsrlib._QSRlib__print_qsrs_available()

## Inputs.
# time increment we are studying
td = 0.5

#- Human state vector: position (xh,yh,hh), and speed (vh,wh)
xh0=0.
yh0=1.
hh0=np.pi/2
vh0 = 1
wh0 = 0.0

hh1 = hh0 + wh0*td
xh1 = xh0 + vh0*np.cos(hh1) * td
yh1 = yh0 + vh0*np.sin(hh1) * td

#- Set of forbidden qsc states: i.e. {(----), (-0--), ...}
# lets create some custom forbiden states
state_forbidden = ['-,-,-,-', '-,-,-,0', '-,-,-,+','-,-,+,-', '-,+,-,-', '0,-,-,-', '+,-,-,-','-,-,0,-']


#- Brute force: we generate a 2D grid of speeds, from them corresponding qtc states, from them we identify valid/invalid space and finally get biggest bounding box

# Init robot state
xr0=0.
yr0=2.
hr0=-np.pi/2
vr0 = 0
wr0 = 0

# how many points we want to test
num_speed_points = 20
# maximum ABSOLUTE robot acceleration m/s^2
max_acc_vr = 1.5
# maximum ABSOLUTE robot angular acceleration rad/s^2
max_acc_wr = np.pi
#plot possible goals from current space
viewSpace=True



# What possible speeds we can achieve
vr_space = np.linspace(-max_acc_vr*td, max_acc_vr*td, num_speed_points) + vr0
wr_space =  np.linspace(-max_acc_wr*td, max_acc_wr*td, num_speed_points) + wr0
# and our 2D speed space
V, W = np.meshgrid(vr_space,wr_space)

# and what posible points we can achieve
Hr1 = hr0 + W*td
Xr1 = xr0 + V*np.cos(Hr1) * td
Yr1 = yr0 + V*np.sin(Hr1) * td

if viewSpace:
    x_plot = Xr1.flatten()
    y_plot = Yr1.flatten()

    xmin=x_plot.min()
    xmin=np.min([xmin,xr0,xh0,xh1])-0.5
    xmax=x_plot.max()
    xmax = np.max([xmax, xr0, xh0,xh1])+0.5
    ymin=y_plot.min()
    ymin = np.min([ymin, yr0, yh0, yh1])-0.5
    ymax=y_plot.max()
    ymax = np.max([ymax, yr0, yh0, yh1])+0.5


    u_plot = np.cos(Hr1).flatten()
    v_plot = np.sin(Hr1).flatten()


    # final robot pose
    qr1=plt.quiver(x_plot, y_plot, u_plot, v_plot, angles='uv', units='xy', scale=2*num_speed_points , color='blue')
    # start robot pose
    qr0 =plt.quiver(xr0, yr0, np.cos(hr0), np.sin(hr0), angles='uv', units='xy', scale=num_speed_points, color='cyan')
    qr00 =plt.quiver(xr0, yr0, np.cos(hr0), np.sin(hr0), angles='uv', units='xy', scale=num_speed_points, edgecolor='k',
               facecolor='None', linewidth=.5)

    # end human pose
    qh1 =plt.quiver(xh1, yh1, np.cos(hh1), np.sin(hh1), angles='uv', units='xy', scale=num_speed_points, color='green')

    # start human pose
    qh0 =plt.quiver(xh0, yh0, np.cos(hh0), np.sin(hh0), angles='uv', units='xy', scale=num_speed_points, color='lightgreen')
    qh00 =plt.quiver(xh0, yh0, np.cos(hh0), np.sin(hh0), angles='uv', units='xy', scale=num_speed_points, edgecolor='k',
               facecolor='None', linewidth=.5)
    
    legend_x=0.92
    legend_y = 0.85
    dec_y = 0.05

    from matplotlib.lines import Line2D
    legend_elements = [Line2D([0], [0], marker='<', color='k', label='start robot pose', markerfacecolor='cyan', markersize=15)]

    legend_elements.append(Line2D([0], [0], marker='<', color='k', label='end robot poses', markerfacecolor='blue',markersize=15))

    legend_elements.append(Line2D([0], [0], marker='<', color='k', label='start human pose', markerfacecolor='lightgreen', markersize=15))

    legend_elements.append(Line2D([0], [0], marker='<', color='k', label='end human pose', markerfacecolor='g', markersize=15))


    ax = plt.gca()
    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymin, ymax])

    ax.set_xlabel('X coord. (m)')
    ax.set_ylabel('Y coord. (m)')
    ax.set_title('Position space according to accels,times,sampling')
    ax.legend(handles=legend_elements, numpoints=1, loc='lower left')
all_states=[]
T = np.zeros_like(Xr1).flatten()

for i in range(0, len(Xr1.flatten()) ):
    xr1=Xr1.flatten()[i]
    yr1=Yr1.flatten()[i]
    hr1 = Hr1.flatten()[i]
    (T[i],state)   = isValidState(xh0,yh0,xh1,yh1,xr0,yr0,xr1,yr1,td, which_qsr, dynammic_args, state_forbidden)

    all_states.append(state)

    print("Robot pos (%3.3f, %3.3f, %3.3f deg) has state %s" %( xr1,yr1,hr1*180.0/np.pi,state))

T = T.reshape(Xr1.shape)



#quick way to generate values in a bigger space
if 0:
    from matplotlib.mlab import griddata
    vr_space_deep = np.linspace(-max_acc_vr*td, max_acc_vr*td, 50*num_speed_points) + vr0
    wr_space_deep =  np.linspace(-max_acc_wr*td, max_acc_wr*td, 50*num_speed_points) + wr0

    Ti = griddata(V.flatten(), W.flatten(), T.flatten(), vr_space_deep,wr_space_deep , interp='linear')
    plt.figure()
    plt.contourf(vr_space_deep, wr_space_deep, Ti,1,cmap='RdYlGn' )
    ax = plt.gca()
    ax.set_xlabel('Linear speed (m/s)')
    ax.set_ylabel('Angular speed (rad/s)')
    ax.set_title('Valid speeds space')


## Output.
# - Range of acceptable speeds for robot (vr_min,vr_max) (wr_min, wr_max)

max_area,min_i,min_j,max_i,max_j = maximalRectangle(T)
allowed_min_v = V[min_i,min_j]
allowed_max_v = V[max_i,max_j]
allowed_min_w = W[min_i,min_j]
allowed_max_w = W[max_i,max_j]


print(allowed_min_v)
print(allowed_max_v)
print(allowed_min_w)
print(allowed_max_w)

plt.figure()
# speed space allowed speeds
plt.contourf(V, W, T,1,cmap='RdYlGn')
ax = plt.gca()
ax.set_xlabel('Linear speed (m/s)')
ax.set_ylabel('Angular speed (rad/s)')
ax.set_title('Valid speeds space')

r = Rectangle((allowed_min_v, allowed_max_w), allowed_max_v-allowed_min_v, allowed_min_w-allowed_max_w, fill=True,hatch="/",alpha=0.1,facecolor='green')

ax.add_artist(r)

ax.text(0.5*(allowed_min_v + allowed_max_v), 0.5*(allowed_max_w + allowed_min_w), 'Allowed speeds subset',
        horizontalalignment='center',
        verticalalignment='center',
        fontsize=30, color='black')

plt.show()