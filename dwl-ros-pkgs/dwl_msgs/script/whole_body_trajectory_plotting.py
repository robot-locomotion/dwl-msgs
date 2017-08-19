#!/usr/bin/python
import numpy as np

from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from matplotlib import rc
rc('font',**{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

import roslib; roslib.load_manifest('dwl_msgs')
import argparse
from dwl_msgs import BagParser as bag_parser


    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts WholeBodyControllerState messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    args = parser.parse_args()
    
    ws_actual_vec, ws_desired_vec, ws_error_vec = bag_parser.extractWholeBodyControllerState(args.bag, args.topic)

    # Plotting the base states
#    num_base_joints = len(base_pos[0]) / 2
#    for i in range(num_base_joints):
#        # Plotting the base position
#        fig = plt.figure(i)
#        ax = fig.add_subplot(111)
#        ax.plot(time,base_pos[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,base_pos[:,2*i], 'r', linewidth=2.5)
#        plt.title('Base Position', fontsize=18, fontweight='bold')
#        plt.ylabel('$z$ $[m]$', {'color':'k', 'fontsize':18})
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':18})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('base_pos_' + str(i) + '.pdf')
#        
#        # Plotting the base velocity
#        fig = plt.figure(num_base_joints*(i+1))
#        ax = fig.add_subplot(111)
#        ax.plot(time,base_vel[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,base_vel[:,2*i], 'r', linewidth=2.5)
#        plt.ylabel('Base Velocity')
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('base_vel_' + str(i) + '.pdf')
#        
#        # Plotting the base acceleration
#        fig = plt.figure(num_base_joints*(i+2))
#        ax = fig.add_subplot(111)
#        ax.plot(time,base_acc[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,base_acc[:,2*i], 'r', linewidth=2.5)
#        plt.ylabel('Base Acceleration')
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('base_acc_' + str(i) + '.pdf')
#        
#    # Plotting the joint states
#    num_joints = len(joint_pos[0]) / 2
#    for i in range(num_joints):
#        fig = plt.figure(i+2*(num_base_joints*num_base_joints))
#        ax = fig.add_subplot(111)
#        ax.plot(time,joint_pos[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,joint_pos[:,2*i], 'r', linewidth=2.5)
#        plt.ylabel('Joint position')
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('joint_pos_' + str(i) + '.pdf')
#        
#        # Plotting the joint velocity
#        fig = plt.figure(num_joints*(i+1)+(num_base_joints*(num_base_joints+2)))
#        ax = fig.add_subplot(111)
#        ax.plot(time,joint_vel[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,joint_vel[:,2*i], 'r', linewidth=2.5)
#        plt.ylabel('Joint Velocity')
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('joint_vel_' + str(i) + '.pdf')
#        
#        # Plotting the joint acceleration
#        fig = plt.figure(num_joints*(i+2)+(num_base_joints*(num_base_joints+2)))
#        ax = fig.add_subplot(111)
#        ax.plot(time,joint_acc[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(time,joint_acc[:,2*i], 'r', linewidth=2.5)
#        plt.ylabel('Joint Acceleration')
#        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        ax.grid(True)
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
#        fig.savefig('joint_acc_' + str(i) + '.pdf')
#        
#    
#    # Plotting the contact states
#    num_contacts = 1#len(joint_state[0])
#    for i in range(num_contacts):
#        fig = plt.figure(i+(num_base_joints*(num_base_joints+2))+num_joints*(num_joints+2))
#        ax = fig.add_subplot(111)
#        actual_x = 3*num_contacts*i
#        actual_y = 3*num_contacts*i+1
#        actual_z = 3*num_contacts*i+2
#        desired_x = 3*(num_contacts*i+1)
#        desired_y = 3*(num_contacts*i+1)+1
#        desired_z = 3*(num_contacts*i+1)+2
#        ax.plot(contact_pos[:,desired_x],contact_pos[:,desired_z]+base_pos[:,2*i+1], 'k', linewidth=2.5)
##        ax.plot(time,contact_pos[:,desired_z]+base_pos[:,2*i+1], 'k', linewidth=2.5)
##        ax.plot(time,contact_pos[:,actual_z]+base_pos[:,2*i]+0.2283, 'r', linewidth=2.5)
#
#        ax.plot(contact_pos[:,desired_x],contact_pos[:,desired_z]+base_pos[:,2*i+1], 'k', linewidth=2.5)
#        ax.plot(contact_pos[:,actual_x],contact_pos[:,actual_z]+base_pos[:,2*i]+0.2283, 'r', linewidth=2.5)
#
#
#        plt.ylabel('$z$ $[m]$', {'color':'k', 'fontsize':16})
#        plt.xlabel('$x$ $[m]$', {'color':'k', 'fontsize':16})
#        plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
#        plt.grid(True)
#        plt.gca().invert_xaxis()
#        ax.spines['right'].set_visible(False)
#        ax.spines['top'].set_visible(False)
#        ax.yaxis.set_ticks_position('left')
#        ax.xaxis.set_ticks_position('bottom')
##         someX, someY = 2, 3
##         currentAxis = plt.gca()
##         plt.gca().add_patch(Rectangle((someX - .05, someY - .05), 1, 1, facecolor="grey"))
#        ax.broken_barh([(0.11, 0.185)] , (-0.5827, 0.15), facecolors='#FFE4B5')
##        ax.set_ylim(-0.5827,-0.3)
#        fig.savefig('contact_pos_' + str(i) + '.pdf')
#        
##         # Plotting the joint velocity
##         fig = plt.figure(num_contacts*(i+1)+(num_base_joints*(num_base_joints+2))+num_joints*(num_joints+2))
##         plt.plot(time,joint_vel[:,2*i+1], '--k', linewidth=4)
##         plt.plot(time,joint_vel[:,2*i], 'r', linewidth=2.5)
##         plt.ylabel('Joint velocity')
##         plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
##         plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
##         fig.tight_layout()
##         fig.savefig('joint_vel_' + str(i) + '.png')
##         
##         # Plotting the joint acceleration
##         fig = plt.figure(num_contacts*(i+2)+(num_base_joints*(num_base_joints+2))+num_joints*(num_joints+2))
##         plt.plot(time,joint_acc[:,2*i+1], '--k', linewidth=4)
##         plt.plot(time,joint_acc[:,2*i], 'r', linewidth=2.5)
##         plt.ylabel('Joint acceleration')
##         plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':16})
##         plt.legend((r'$Desired$', r'$Executed$'), shadow = True, loc = (0.8, 0))
##         fig.tight_layout()
##         fig.savefig('joint_acc_' + str(i) + '.png')
