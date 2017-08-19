#!/usr/bin/python
import roslib; roslib.load_manifest('dwl_msgs')

import dwl
from dwl_msgs.msg import WholeBodyState, WholeBodyStateInterface
import numpy as np
import rosbag



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
        ws_list = []
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
            

                # Construct an instance of the WholeBodyState class, which wraps the C++ class.
                ws = dwl.WholeBodyState()

                # Setting up the DoF
                ws.setJointDoF(num_joints)

                # Getting the time
                ws.setTime(time)

                # Getting the base states
                base_pos = np.array([msg.base[dwl.LX].position,
                                     msg.base[dwl.LY].position,
                                     msg.base[dwl.LZ].position])
                base_rpy = np.array([msg.base[dwl.AX].position,
                                     msg.base[dwl.AY].position,
                                     msg.base[dwl.AZ].position])
                base_vel = np.array([msg.base[dwl.LX].velocity,
                                     msg.base[dwl.LY].velocity,
                                     msg.base[dwl.LZ].velocity])
                base_omega = np.array([msg.base[dwl.AX].velocity,
                                       msg.base[dwl.AY].velocity,
                                       msg.base[dwl.AZ].velocity])
                base_acc = np.array([msg.base[dwl.LX].velocity,
                                     msg.base[dwl.LY].velocity,
                                     msg.base[dwl.LZ].velocity])
                base_omegad = np.array([msg.base[dwl.AX].velocity,
                                        msg.base[dwl.AY].velocity,
                                        msg.base[dwl.AZ].velocity])
                ws.setBasePosition_W(base_pos)
                ws.setBaseRPY_W(base_rpy)
                ws.setBaseVelocity_W(base_vel)
                ws.setBaseAngularVelocity_W(base_omega)
                ws.setBaseAcceleration_W(base_acc)
                ws.setBaseAngularAcceleration_W(base_omegad)


                # Getting the joint states
                for i in range(num_joints):
                    ws.setJointPosition(msg.joints[i].position, i)
                    ws.setJointVelocity(msg.joints[i].velocity, i)
                    ws.setJointAcceleration(msg.joints[i].acceleration, i)
                    ws.setJointEffort(msg.joints[i].effort, i)

                # Getting the contact states
                for i in range(num_contacts):
                    name = msg.contacts[i].name
                    contact_pos = np.array([msg.contacts[i].position.x,
                                            msg.contacts[i].position.y,
                                            msg.contacts[i].position.z])
                    contact_vel = np.array([msg.contacts[i].velocity.x,
                                            msg.contacts[i].velocity.y,
                                            msg.contacts[i].velocity.z])
                    contact_acc = np.array([msg.contacts[i].acceleration.x,
                                            msg.contacts[i].acceleration.y,
                                            msg.contacts[i].acceleration.z])
                    contact_wrc = np.array([msg.contacts[i].wrench.torque.x,
                                            msg.contacts[i].wrench.torque.y,
                                            msg.contacts[i].wrench.torque.z,
                                            msg.contacts[i].wrench.force.x,
                                            msg.contacts[i].wrench.force.y,
                                            msg.contacts[i].wrench.force.z])
                    ws.setContactPosition_B(name, contact_pos)
                    ws.setContactVelocity_B(name, contact_vel)
                    ws.setContactAcceleration_B(name, contact_acc)
                    ws.setContactWrench_B(name, contact_wrc)

                # Appeding the whole-state to the list
                ws_list.append(ws)

                n += 1
    print('Loaded ' + str(n) + ' whole-body controller states from the bagfile.')
    
    return ws_list



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
        ws_actual_list = []
        ws_desired_list = []
        ws_errore_list = []
        for (topic, msg, ts) in bag.read_messages(topics=str(topic)):
            # Recording the data in the desired time range
            if (ts.to_sec() >= starting_time and ts.to_sec() <= ending_time):
                # Defined the desired, actual and error whole-body states
                ws_actual = dwl.WholeBodyState()
                ws_desired = dwl.WholeBodyState()
                ws_error = dwl.WholeBodyState()
                
                wbi = WholeBodyStateInterface()
                wbi.writeFromMessage(ws_actual, msg.actual)
                wbi.writeFromMessage(ws_desired, msg.desired)
                wbi.writeFromMessage(ws_error, msg.error)
                
                # Appeding the whole-state to the list
                ws_actual_list.append(ws_actual)
                ws_desired_list.append(ws_desired)
                ws_error_list.append(ws_error)

                n += 1
    print('Loaded ' + str(n) + ' whole-body controller states from the bagfile.')
    
    return ws_actual_list, ws_desired_list, ws_error_list