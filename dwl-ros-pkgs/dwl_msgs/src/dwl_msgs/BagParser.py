#!/usr/bin/python
import roslib; roslib.load_manifest('dwl_msgs')

import rosbag
from dwl_msgs.msg import WholeBodyState
import numpy as np


def extractWholeBodyState(bagfile, topic):
    print('Extracting the WholeBodyState message in topic' + topic +
          ' from rosbag ' + bagfile)
    
    n = 0
    state_list = list()
    initial_time = 0
    with rosbag.Bag(bagfile, 'r') as bag:
        initial_time = 0.
        duration = 0.
        starting_time = bag.get_start_time() + initial_time
        if (duration == 0.0):
            ending_time = bag.get_end_time()
            duration = ending_time - starting_time
        else:
            ending_time = starting_time + duration
        
        # Recording the whole-body state trajectory
        time_list = list()
        base_pos_list = list()
        base_vel_list = list()
        base_acc_list = list()
        joint_pos_list = list()
        joint_vel_list = list()
        joint_acc_list = list()
        contact_list = list()
        for (topic, msg, ts) in bag.read_messages(topics=str(topic)):
            # Recording the data in the desired time range
            if (ts.to_sec() >= starting_time and ts.to_sec() <= ending_time):
                # Setting the floating-base system DoF
                num_base_joints = len(msg.base)
                num_joints = len(msg.joints)
                num_contacts = len(msg.contacts)
            
                # Getting the current time
                if (n == 0):
                    initial_time = ts.to_sec()
                time = ts.to_sec() - initial_time
            
                # Defining the state lists
                base_pos = list()
                base_vel = list()
                base_acc = list()
                joint_pos = list()
                joint_vel = list()
                joint_acc = list()
                contact_state = list()

                # Getting the base states
                for i in range(num_base_joints):
                    base_pos.append(msg.base[i].position)
                    base_vel.append(msg.base[i].velocity)
                    base_acc.append(msg.base[i].acceleration)
                
                # Getting the joint states
                for i in range(num_joints):
                    joint_pos.append(msg.joints[i].position)
                    joint_vel.append(msg.joints[i].velocity)
                    joint_acc.append(msg.joints[i].acceleration)
                                        
                # Getting the contact states
                for i in range(num_contacts):
                    contact_state.append(msg.contacts[i].position.x)
                    contact_state.append(msg.contacts[i].position.y)
                    contact_state.append(msg.contacts[i].position.z)

                base_pos_list.append(base_pos)
                base_vel_list.append(base_vel)
                base_acc_list.append(base_acc)
                joint_pos_list.append(joint_pos)
                joint_vel_list.append(joint_vel)
                joint_acc_list.append(joint_acc)
                contact_list.append(contact_state)

                n += 1
    print('Loaded ' + str(n) + ' whole-body controller states from the bagfile.')
    
    # Getting the time info
    for i in range(n):
        time_list.append(i * duration/n)
    
    
    time_vec = np.array(time_list)
    base_pos_vec = np.array(base_pos_list)
    base_vel_vec = np.array(base_vel_list)
    base_acc_vec = np.array(base_acc_list)
    joint_pos_vec = np.array(joint_pos_list)
    joint_vel_vec = np.array(joint_vel_list)
    joint_acc_vec = np.array(joint_acc_list)
    contact_vec = np.array(contact_list)
            
    return time_vec, base_pos_vec, base_vel_vec, base_acc_vec, joint_pos_vec, joint_vel_vec, joint_acc_vec, contact_vec



def extractWholeBodyControllerState(bagfile, topic):
    print('Extracting the WholeBodyControllerState message in topic' + topic +
          ' from rosbag ' + bagfile)

    n = 0
    state_list = list()
    initial_time = 0
    with rosbag.Bag(bagfile, 'r') as bag:
        initial_time = 0.
        duration = 0.
        starting_time = bag.get_start_time() + initial_time
        if (duration == 0.0):
            ending_time = bag.get_end_time()
            duration = ending_time - starting_time
        else:
            ending_time = starting_time + duration
        
        # Recording the whole-body state trajectory
        time_list = list()
        base_pos_list = list()
        base_vel_list = list()
        base_acc_list = list()
        joint_pos_list = list()
        joint_vel_list = list()
        joint_acc_list = list()
        contact_list = list()
        for (topic, msg, ts) in bag.read_messages(topics=str(topic)):
            # Recording the data in the desired time range
            if (ts.to_sec() >= starting_time and ts.to_sec() <= ending_time):
                # Setting the floating-base system DoF
                num_base_joints = len(msg.actual.base)
                num_joints = len(msg.actual.joints)
                num_contacts = len(msg.actual.contacts)
            
                # Getting the current time
                if (n == 0):
                    initial_time = ts.to_sec()
                time = ts.to_sec() - initial_time
            
                # Defining the state lists
                base_pos = list()
                base_vel = list()
                base_acc = list()
                joint_pos = list()
                joint_vel = list()
                joint_acc = list()
                contact_state = list()

                # Getting the base states
                for i in range(num_base_joints):
                    base_pos.append(msg.actual.base[i].position)
                    base_pos.append(msg.desired.base[i].position)
                    base_vel.append(msg.actual.base[i].velocity)
                    base_vel.append(msg.desired.base[i].velocity)
                    base_acc.append(msg.actual.base[i].acceleration)
                    base_acc.append(msg.desired.base[i].acceleration)
                
                # Getting the joint states
                for i in range(num_joints):
                    joint_pos.append(msg.actual.joints[i].position)
                    joint_pos.append(msg.desired.joints[i].position)
                    joint_vel.append(msg.actual.joints[i].velocity)
                    joint_vel.append(msg.desired.joints[i].velocity)
                    joint_acc.append(msg.actual.joints[i].acceleration)
                    joint_acc.append(msg.desired.joints[i].acceleration)
                                        
                # Getting the contact states
                for i in range(num_contacts):
                    contact_state.append(msg.actual.contacts[i].position.x)
                    contact_state.append(msg.actual.contacts[i].position.y)
                    contact_state.append(msg.actual.contacts[i].position.z)
                    contact_state.append(msg.desired.contacts[i].position.x)
                    contact_state.append(msg.desired.contacts[i].position.y)
                    contact_state.append(msg.desired.contacts[i].position.z)

                base_pos_list.append(base_pos)
                base_vel_list.append(base_vel)
                base_acc_list.append(base_acc)
                joint_pos_list.append(joint_pos)
                joint_vel_list.append(joint_vel)
                joint_acc_list.append(joint_acc)
                contact_list.append(contact_state)

                n += 1
    print('Loaded ' + str(n) + ' whole-body controller states from the bagfile.')
    
    # Getting the time info
    for i in range(n):
        time_list.append(i * duration/n)
    
    
    time_vec = np.array(time_list)
    base_pos_vec = np.array(base_pos_list)
    base_vel_vec = np.array(base_vel_list)
    base_acc_vec = np.array(base_acc_list)
    joint_pos_vec = np.array(joint_pos_list)
    joint_vel_vec = np.array(joint_vel_list)
    joint_acc_vec = np.array(joint_acc_list)
    contact_vec = np.array(contact_list)

    return time_vec, base_pos_vec, base_vel_vec, base_acc_vec, joint_pos_vec, joint_vel_vec, joint_acc_vec, contact_vec