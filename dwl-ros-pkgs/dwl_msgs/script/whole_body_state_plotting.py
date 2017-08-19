#!/usr/bin/python
import dwl
import numpy as np
from dwl_msgs import BagParser as bag_parser
import roslib; roslib.load_manifest('dwl_msgs')
import argparse
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
    base_RPY = [[k.getBaseRPY_W()[i] for k in ws_vec] for i in range(0,3)]
    base_pos = [[k.getBasePosition_W()[i] for k in ws_vec] for i in range(0,3)]
    base_omega = [[k.getBaseAngularVelocity_W()[i] for k in ws_vec] for i in range(0,3)]
    base_vel = [[k.getBaseVelocity_W()[i] for k in ws_vec] for i in range(0,3)]
    base_omaged = [[k.getBaseAngularAcceleration_W()[i] for k in ws_vec] for i in range(0,3)]
    base_acc = [[k.getBaseAcceleration_W()[i] for k in ws_vec] for i in range(0,3)]
    
    # Joint states
    num_joints = ws_vec[0].getJointDoF()
    joint_pos = [[k.getJointPosition()[i] for k in ws_vec] for i in range(0,num_joints)]
    joint_vel = [[k.getJointVelocity()[i] for k in ws_vec] for i in range(0,num_joints)]
    joint_acc = [[k.getJointAcceleration()[i] for k in ws_vec] for i in range(0,num_joints)]
    joint_eff = [[k.getJointEffort()[i] for k in ws_vec] for i in range(0,num_joints)]
    
    # Contact states
    contact_names = [c for c in ws_vec[0].getContactPosition_B()]
    contact_pos = dict([]);  contact_vel = dict([]);  contact_acc = dict([])
    contact_wrc = dict([])
    for c in contact_names:
        contact_pos[c] = [[k.getContactPosition_B(c)[i] for k in ws_vec] for i in range(0,3)]
        contact_vel[c] = [[k.getContactVelocity_B(c)[i] for k in ws_vec] for i in range(0,3)]
        contact_acc[c] = [[k.getContactAcceleration_B(c)[i] for k in ws_vec] for i in range(0,3)]
        contact_wrc[c] = [[k.getContactAcceleration_B(c)[i] for k in ws_vec] for i in range(0,6)]

    #fig, ax = plt.subplots()
    fig = plt.figure(1)
    ax = fig.add_subplot(111)
    ax.plot(time, base_pos[dwl.X], 'r', linewidth=2.5)
    #ax.plot(time, joint_pos[0], 'r', linewidth=2.5)
    #ax.plot(time, contact_pos['lh_foot'][dwl.X], 'r', linewidth=2.5)
    plt.show()

    # Plotting the base states
#    num_base_joints = len(base_pos[0]) / 2
#    for i in range(num_base_joints):
#        # Plotting the base position
#        fig = plt.figure(i)
#        ax = fig.add_subplot(111)
##        ax.plot(time,base_pos[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,base_pos[:,2*i], 'r', linewidth=2.5)
#        plt.title('Base Position', fontsize=18, fontweight='bold')
#        plt.ylabel('$z$ $[m]$', {'color':'k', 'fontsize':18})
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':18})
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('base_pos_' + str(i) + '.pdf')
