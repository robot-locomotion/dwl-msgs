#!/usr/bin/python
import dwl
import numpy as np
from dwl_msgs import BagParser as bag_parser
import roslib; roslib.load_manifest('dwl_msgs')
import argparse
import matplotlib as mpl
import matplotlib.pyplot as plt

   
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts WholeBodyState messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    args = parser.parse_args()
    
    # Extrating the whole-body trajectory
    ws_vec = bag_parser.extractWholeBodyState(args.bag, args.topic)
    
    # Time vector
    time = [[k.getTime()] for k in ws_vec]
    
    # Base states
    base_RPY = [[k.getBaseRPY_W()[i] for k in ws_vec] for i in range(3)]
    base_pos = [[k.getBasePosition_W()[i] for k in ws_vec] for i in range(3)]
    base_omega = [[k.getBaseAngularVelocity_W()[i] for k in ws_vec] for i in range(3)]
    base_vel = [[k.getBaseVelocity_W()[i] for k in ws_vec] for i in range(3)]
    base_omaged = [[k.getBaseAngularAcceleration_W()[i] for k in ws_vec] for i in range(3)]
    base_acc = [[k.getBaseAcceleration_W()[i] for k in ws_vec] for i in range(3)]
    
    # Joint states
    num_joints = ws_vec[0].getJointDoF()
    joint_pos = [[k.getJointPosition()[i] for k in ws_vec] for i in range(num_joints)]
    joint_vel = [[k.getJointVelocity()[i] for k in ws_vec] for i in range(num_joints)]
    joint_acc = [[k.getJointAcceleration()[i] for k in ws_vec] for i in range(num_joints)]
    joint_eff = [[k.getJointEffort()[i] for k in ws_vec] for i in range(num_joints)]
    
    # Contact states
    contact_names = [c for c in ws_vec[0].getContactPosition_B()]
    contact_pos = dict([]);  contact_vel = dict([]);  contact_acc = dict([])
    contact_wrc = dict([])
    for c in contact_names:
        contact_pos[c] = [[k.getContactPosition_B(c)[i] for k in ws_vec] for i in range(3)]
        contact_vel[c] = [[k.getContactVelocity_B(c)[i] for k in ws_vec] for i in range(3)]
        contact_acc[c] = [[k.getContactAcceleration_B(c)[i] for k in ws_vec] for i in range(3)]
        contact_wrc[c] = [[k.getContactWrench_B(c)[i] for k in ws_vec] for i in range(6)]

    #fig, ax = plt.subplots()
#    fig = plt.figure(1)
#    ax = fig.add_subplot(111)
    #ax.plot(time, base_pos[dwl.X], 'r', linewidth=2.5)
    #ax.plot(time, joint_pos[0], 'r', linewidth=2.5)
    #ax.plot(time, contact_pos['lh_foot'][dwl.X], 'r', linewidth=2.5)
#    ax.plot(time, contact_wrc['lf_foot'][dwl.LZ], 'r', linewidth=2.5)
#    plt.show()

    # Plotting the base position
    fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, sharex=True)
    ax0.plot(time, base_pos[dwl.X], 'b', linewidth=2.5)
    ax0.set_title('Base Position', fontsize=18)
    ax0.set_ylabel('$x$ $[m]$', {'color':'k', 'fontsize':18})
    ax0.grid(True)
    ax0.spines['right'].set_visible(False)
    ax0.spines['top'].set_visible(False)

    ax1.plot(time, base_pos[dwl.Y], 'b', linewidth=2.5)
    ax1.set_ylabel('$y$ $[m]$', {'color':'k', 'fontsize':18})
    ax1.grid(True)
    ax1.spines['right'].set_visible(False)
    ax1.spines['top'].set_visible(False)

    ax2.plot(time, base_pos[dwl.Z], 'b', linewidth=2.5)
    ax2.set_ylabel('$z$ $[m]$', {'color':'k', 'fontsize':18})
    ax2.set_xlabel('$t$ $[s]$', {'color':'k', 'fontsize':18})
    ax2.grid(True)
    ax2.spines['right'].set_visible(False)
    ax2.spines['top'].set_visible(False)
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(base_pos[dwl.X], base_pos[dwl.Y], 'b', linewidth=2.5)
    ax.set_title('Base Position', fontsize=18)
    ax.set_ylabel('$y$ $[m]$', {'color':'k', 'fontsize':18})
    ax.set_xlabel('$x$ $[m]$', {'color':'k', 'fontsize':18})
    ax.grid(True)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    plt.show()

    
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('base_pos_' + str(i) + '.pdf')
