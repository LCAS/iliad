import rosbag
import numpy as np
import matplotlib.pyplot as plt



def parse_report_from_rosbag(topic_name,recordedBag):
    x = []
    y = []
    tetha = []
    phi = []
    cn = []
    sn = []
    st = []

    for topic, msg, t in recordedBag.read_messages(topics=[topic_name]):
        x.append(msg.state.position_x)
        y.append(msg.state.position_y)
        tetha.append(msg.state.orientation_angle)
        phi.append(msg.state.steering_angle)
        cn.append(msg.traj_chunk_sequence_num)
        sn.append(msg.traj_step_sequence_num)
        st.append(msg.status)

    x = np.array(x)
    y = np.array(y)
    tetha = np.array(tetha)
    phi = np.array(phi)
    cn = np.array(cn)
    sn = np.array(sn)
    st = np.array(st)

    return (x, y, tetha, phi, cn, sn, st)


def parse_trajectory_from_rosbag(topic_name,recordedBag):
    """

    :param topic_name:
    :param recordedBag:
    :return: x, y, tetha (robot orientation), phi (wheel orient), cn chunck number, sn step number
    """
    x = []
    y = []
    tetha = []
    phi = []
    v = []
    w = []
    cn = []
    sn  = []

    print("reading ["+topic_name+"]")
    for topic, msg, t in recordedBag.read_messages(topics=[topic_name]):
        print("   num chunks:"+ str(len(msg.chunks)))
        for chunk_i in msg.chunks:
            si = 0
            print("      num steps:" + str(len(chunk_i.steps)))
            for step_i in chunk_i.steps:
                x.append(step_i.state.position_x)
                y.append(step_i.state.position_y)
                tetha.append(step_i.state.orientation_angle)
                phi.append(step_i.state.steering_angle)
                v.append(step_i.velocities.tangential)
                w.append(step_i.velocities.steering)
                cn.append(chunk_i.sequence_num)
                sn.append(si)
                si = si + 1


    x = np.array(x)
    y = np.array(y)
    tetha = np.array(tetha)
    phi = np.array(phi)
    v = np.array(v)
    w = np.array(w)
    cn = np.array(cn)
    sn = np.array(sn)

    return (x,y,tetha,phi,v,w,cn, sn)

def computeSpeeds(x, y, th, phi, time_step):
    dx = np.gradient(x, time_step)
    dy = np.gradient(y, time_step)
    dphi = np.gradient(phi, time_step)

    th1 = th
    th1[th == 0] = 1.0
    v = dy / np.sin(th1)
    v[th == 0] = dx[th == 0]

    w = dphi
    return (w, v)


bagURI = '/home/manolofc/workspace/iliad_ws/reactive_nav_fail_2.bag'

bag = rosbag.Bag(bagURI)

topics = bag.get_type_and_topic_info()[1].keys()
types = bag.get_type_and_topic_info()[1].values()

print("Recorded topics are:")
print '\n'.join(topics)

# what mpc covered
print("Executed")
(x_exec, y_exec, tetha_exec, phi_exec,cn_exec, sn_exec, st_exec) = parse_report_from_rosbag('/robot4/control/controller/reports_mpc',bag)
# last good point
i = len(st_exec[st_exec!=2]) - 1

# last properly executed point was
last_cn = cn_exec[i]
last_sn = sn_exec[i]

# this last executed point should correspond to index
chunk_size = 10
last_index = last_cn * chunk_size + last_sn

# this shows chunck/step execution and state ...
if False:
    plt.figure()
    plt.plot(cn_exec[0:i])
    plt.plot(sn_exec[0:i])
    plt.plot(st_exec[0:i],'.r')
    plt.show()


# what we got from planner
print("From planner")
(x,y,tetha,phi,v,w,cn, sn) = parse_trajectory_from_rosbag('/robot4/control/controller/trajectories', bag)

# this shows chunck/step execution  ...
if False:
    i = len(x)
    plt.figure()
    plt.plot(cn[0:i])
    plt.plot(sn[0:i])



# what we send to mpc
print("to MPC")
(x_mpc, y_mpc,tetha_mpc,phi_mpc,v_mpc,w_mpc,cn_mpc, sn_mpc) = parse_trajectory_from_rosbag('/robot4/control/controller/trajectories_mpc', bag)

# this shows chunck/step execution  ...
if False:
    i = len(x_mpc)
    plt.figure()
    plt.plot(cn_mpc[0:i])
    plt.plot(sn_mpc[0:i])





#plt.show()

showPoses = False

if showPoses:
    # see three trajs together
    plt.plot(x_mpc[0:last_index],y_mpc[0:last_index],'.', label='HALP trajectory')
    plt.plot(x_mpc[last_index],y_mpc[last_index],'.k')

    plt.plot(x[0:last_index],y[0:last_index],'xr', label='ORU trajectory')
    plt.plot(x[last_index],y[last_index],'xk')

    plt.plot(x_exec[0:i],y_exec[0:i],'og', label='Executed trajectory')
    plt.plot(x_exec[i],y_exec[i],'ok')
    plt.legend()

showArrows = True
if showArrows:
    # see three trajs together
    mpc_u = v_mpc * np.sin(tetha_mpc+phi_mpc+w_mpc*0.06)
    mpc_v = v_mpc * np.cos(tetha_mpc+phi_mpc+w_mpc*0.06)
    plt.quiver(x_mpc[0:last_index], y_mpc[0:last_index],mpc_u[0:last_index],mpc_v[0:last_index], color='b',width=0.005, label='HALP trajectory')
    plt.quiver(x_mpc[last_index], y_mpc[last_index], mpc_u[last_index],mpc_v[last_index],width=0.005)

    _u = v * np.cos(tetha+phi+w*0.06)
    _v = v * np.sin(tetha+phi+w*0.06)
    plt.quiver(x[0:last_index], y[0:last_index],_u[0:last_index],_v[0:last_index], color='r',width=0.005, label='ORU trajectory')
    plt.quiver(x[last_index], y[last_index],_u[last_index],_v[last_index],width=0.005)

    (w_exec, v_exec) = computeSpeeds(x_exec,y_exec,tetha_exec,phi_exec,0.06)

    exec_u = v_exec * np.cos(tetha_exec+phi_exec+w_exec*0.06)
    exec_v = v_exec * np.sin(tetha_exec+phi_exec+w_exec*0.06)
    plt.quiver(x_exec[0:i], y_exec[0:i],exec_u[0:i],exec_v[0:i], color='g',width=0.005, label='Executed trajectory')
    plt.quiver(x_exec[i], y_exec[i],exec_u[i],exec_v[i], width=0.005)
    plt.legend()

plt.show()

