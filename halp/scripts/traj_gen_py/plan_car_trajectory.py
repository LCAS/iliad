"""
This library should be a reimplementation of the matlab version


"""

import numpy as np


def poly3(qi,qf,vi,vf,t):
    """
     Using 3rd order polynomial generate position, velocity and acceleration profiles that satisfy the initial and final position and velocity constraints.

    : param qi:     initial position
    : param qf:     final position
    : param vi:     initial velocity
    : param vf:     final velocity
    : param t:      np.arange(ti,tf+time_step,time_step)

    :return:
        q    - position profile
        dq   - velocity profile
        ddq  - acceleration profile

    Example
        (q, dq, ddq) = poly3(5, 30, 0, 0, np.arange(0,10.01,0.01))
    """
    t = t.flatten()

    ti = t[0]
    tf = t[-1]

    N = len(t)
    I = np.ones((N,1))
    Z = np.zeros((N,1))

    b = np.vstack((qi,vi,qf,vf))

    A =np.array([ [   (ti**3),   (ti**2),   ti,  1],
                  [ 3*(ti**2),  2*ti,        1,  0],
                  [   (tf**3),   (tf**2),   tf,  1],
                  [ 3*(tf**2),  2*tf,        1,  0] ])

    p = np.linalg.solve(A, b)                     # p = [a b c d]'

    # print((t**3).shape)
    # print((t**2).shape)
    # print((t).shape)
    # print((I).shape)

    # need all to have same number of dimmensions
    t = t.reshape(t.shape[0],1)

    tt = np.hstack((t**3, t**2, t, I))
    q = tt.dot(p)                    # position

    tt = np.hstack(( 3*(t ** 2), 2*t, I,Z))
    dq = tt.dot(p)                   # velocity


    # print "\ntt"
    # print tt
    # print "\np"
    # print p
    # print "\ndq"
    # print dq


    tt = np.hstack(( 6*t, 2*I, Z, Z))
    ddq = tt.dot(p)                  # acceleration
    return (q, dq, ddq)
    ### EOF poly3
    # --------------------------------------------------------------------------------





def path_poly3(s0, s1, k, tspan):
    """
    Path planning for a car-like vehicle using cubic polynomials

    :param s0:     initial state
    :param s0:     final state 
    :param k:      free parameters (k is a vector with two positive entries)
    :param tspan:  time span (e.g., tspan = 0:0.1:10)

    :return:
    P,dP,ddP,dddP - motion profile: position, velocity, acceleration and jerk
    q             - arc length profile (between 0:1)
    dq            - speed on the path


    Note:
    -----
    s = (x,y,theta,phi), but phi is not considered in the analysis

    """

    (q,dq, ddq) = poly3(0, 1, 0, 0, tspan) # generate speed profile on the path
    q = q.flatten()
    dq = dq.flatten()
    s0 = s0.flatten()
    s1 = s1.flatten()
    k = k.flatten()

    t0 = 0
    t1 = 1
    A = np.array([[  1, t0,  t0**2,    t0**3],
                  [  1, t1,  t1**2,    t1**3],
                  [  0,  1,  2*t0,  3*(t0**2)],
                  [  0,  1,  2*t1,  3*(t1**2)] ])

    bx = np.array([ [s0[0]],
                    [s1[0]],
          [k[0]*np.cos(s0[2])],
          [k[1]*np.cos(s1[2])] ])

    by = np.array([ [s0[1]],
                    [s1[1]],
          [k[0]*np.sin(s0[2])],
          [k[1]*np.sin(s1[2])] ])

    b = np.hstack((bx,by))

    tmp =  np.linalg.solve(A, b )


    # print "\ntmp"
    # print tmp
    # print "\nA"
    # print A
    # print "\nb"
    # print b

    px = tmp[:,0]
    py = tmp[:,1]

    x    = px[0] + px[1]*q +   px[2]*(q**2) +   px[3]*(q**3)
    dx   =         px[1]   + 2*px[2]*q      + 3*px[3]*(q**2)
    ddx  =                   2*px[2]        + 6*px[3]*q
    dddx =                                    6*px[3]*np.ones((len(q),1)).flatten()

    y    = py[0] + py[1]*q +   py[2]*(q**2) +   py[3]*(q**3)
    dy   =         py[1]   + 2*py[2]*q      + 3*py[3]*(q**2)
    ddy  =                   2*py[2]        + 6*py[3]*q
    dddy =                                    6*py[3]*np.ones((len(q),1)).flatten()

    P    = np.vstack((x,    y))
    dP   = np.vstack((dx,   dy))
    ddP  = np.vstack((ddx,  ddy))
    dddP = np.vstack((dddx, dddy))



    return (P, dP, ddP, dddP, q, dq)
    # ........................................................................................



def flat_outputs(P,dP,ddP,dddP,L):
    """
    Flat outputs for a car-like vehicle (only forward motion is considered) (formed in function path_poly3)

    :param P:      position
    :param dP:     velocity
    :param ddP:    acceleration
    :param dddP:   jerk
    :param L:      distance between the rear and front axes

    :return:
         state   - evolution of the state variables
         control - evolution of the control inptus
                   u(1) - linear velocity
                   u(2) - steering rate
    """
    #
    # print "\nx"
    # print P[0,:]
    # print "\ny"
    # print P[1,:]
    # print "\ndx"
    # print dP[0,:]
    # print "\ndy"
    # print dP[1,:]
    # print "\nddx"
    # print ddP[0,:]
    # print "\nddy"
    # print ddP[1,:]
    # print "\ndddx"
    # print dddP[0,:]
    # print "\ndddy"
    # print dddP[1,:]

    N = P.shape[1]  # number of points
    control = np.zeros((2,N))
    state = np.zeros((4,N))
    for i in range(0,N):
        # ------------------------------
        # get input
        # ------------------------------
        x    = P[0,i]
        y    = P[1,i]
        dx   = dP[0,i]
        dy   = dP[1,i]
        ddx  = ddP[0,i]
        ddy  = ddP[1,i]
        dddx = dddP[0,i]
        dddy = dddP[1,i]
        # ------------------------------


        v = np.sqrt((dx**2) + (dy**2))

        num = ( ( (dddy*dx) - (dddx*dy) ) * (v**2)  ) - ( 3 * ( (ddy*dx) - (ddx*dy) ) * (  (dx*ddx) + (dy*ddy)  ) )
        den = (v**6) + ( (L**2) * ( (ddy*dx) - (ddx*dy) )**2 )
        w = L*v*num/den

        num = L*( (ddy*dx) - (ddx*dy) )
        den = v**3

        # ------------------------------
        # form output
        # ------------------------------
        state[0,i] = x
        state[1,i] = y
        state[2,i] = np.arctan2(dy/v,dx/v)
        state[3,i] = np.arctan(num/den)
    #keyboard
        control[0,i] = v
        control[1,i] = w
        # ------------------------------



    return (state,control)
    # EOF flat_outputs
    # --------------------------------------------------------------------------------


def plan_car_trajectory(L,s0,s1,kGains,tspan,dt):
    """
    Plan a trajectory for a car-like vehicle
        
    :param L:      distance between the rear and front axes
    :param s0:     initial state
    :param s0:     final state
    :param kGains: free parameters (k is a vector with two positive entries)
    :param tspan:  time span (e.g., tspan = 0:0.1:10)
    :param dt:     time step
    
    :return:
    state   - evolution of the state variables 
    control - evolution of the control inptus
          u(1) - linear velocity 
          u(2) - steering rate    
    """

    (P,dP,ddP,dddP,q,dq) = path_poly3(s0,s1,kGains, tspan)
    (state,control)      = flat_outputs(P,dP,ddP,dddP,L)

    control[0,:] = control[0,:]*dq # derivative w.r.t. time
    control[1,:] = control[1,:]*dq # derivative w.r.t. time

    control[1,0]= (state[3,0] - s0[3]) / dt # workaround for the bug in trajectory generation

    return (state,control)
    # ........................................................................................


def output_trajectory(state_init, state, control, file_name):
    """
    Output state and control trajectories to ASCII file

    :param state_init:  initial state
    :param state:       state trajectory
    :param control:     control trajectory
    :param file_name:   file name where to store the trajectory
    :return:
    """

    #OUT =      X           Y           theta       phi         v             w
    out = np.vstack((state, control))
    control_init = np.zeros((2, 1))
    out_init = np.vstack((state_init,control_init))
    out = np.hstack((out_init,out))

    np.savetxt(file_name, out.transpose(), delimiter=' ',fmt="%2.14f",)
    # ........................................................................................