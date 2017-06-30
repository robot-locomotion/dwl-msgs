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
    Extracts WholeBodyState messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    args = parser.parse_args()
    
    time, base_pos, base_vel, base_acc, joint_pos, joint_vel, joint_acc, contact_pos = bag_parser.extractWholeBodyState(args.bag, args.topic)

    # Plotting the base states
    num_base_joints = len(base_pos[0]) / 2
    for i in range(num_base_joints):
        # Plotting the base position
        fig = plt.figure(i)
        ax = fig.add_subplot(111)
#        ax.plot(time,base_pos[:,2*i+1], 'k', linewidth=2.5)
        ax.plot(time,base_pos[:,2*i], 'r', linewidth=2.5)
        plt.title('Base Position', fontsize=18, fontweight='bold')
        plt.ylabel('$z$ $[m]$', {'color':'k', 'fontsize':18})
        plt.xlabel('$t$ $[s]$', {'color':'k', 'fontsize':18})
        ax.grid(True)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        fig.savefig('base_pos_' + str(i) + '.pdf')
