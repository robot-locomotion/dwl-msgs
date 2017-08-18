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
    
    ws_vec = bag_parser.extractWholeBodyState(args.bag, args.topic)

    time = []
    base_pos_AX = []
    base_pos_AY = []
    base_pos_AZ = []
    base_pos_LX = []
    base_pos_LY = []
    base_pos_LZ = []
    for ws in ws_vec:
        time.append(ws.getTime());
        base_pos_AX.append(ws.getBaseRPY_W()[dwl.X]);
        base_pos_AY.append(ws.getBaseRPY_W()[dwl.Y]);
        base_pos_AZ.append(ws.getBaseRPY_W()[dwl.Z]);
        base_pos_LX.append(ws.getBasePosition_W()[dwl.X]);
        base_pos_LY.append(ws.getBasePosition_W()[dwl.Y]);
        base_pos_LZ.append(ws.getBasePosition_W()[dwl.Z]);


    #fig, ax = plt.subplots()
    fig = plt.figure(1)
    ax = fig.add_subplot(111)
#   ax.plot(time,base_pos[:,2*i+1], 'k', linewidth=2.5)
    #ax.plot(np.array(time_vec),np.array(base_pos_x), 'r', linewidth=2.5)
    ax.plot(time, base_pos_LX, 'r', linewidth=2.5)
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
