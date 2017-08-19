#!/usr/bin/python
import roslib; roslib.load_manifest('dwl_msgs')

import dwl
from dwl_msgs.msg import WholeBodyState
import numpy as np


class WholeBodyStateInterface():
    def __init__(self):
        self.is_system_ = false

    
    def writeFromMessage(self, state, msg):
        # Setting the floating-base system DoF
        num_joints = len(msg.joints)
        num_contacts = len(msg.contacts)
        
        # Setting up the DoF
        state.setJointDoF(num_joints)

        # Getting the time
        state.setTime(time)

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
        state.setBasePosition_W(base_pos)
        state.setBaseRPY_W(base_rpy)
        state.setBaseVelocity_W(base_vel)
        state.setBaseAngularVelocity_W(base_omega)
        state.setBaseAcceleration_W(base_acc)
        state.setBaseAngularAcceleration_W(base_omegad)


        # Getting the joint states
        for i in range(num_joints):
            state.setJointPosition(msg.joints[i].position, i)
            state.setJointVelocity(msg.joints[i].velocity, i)
            state.setJointAcceleration(msg.joints[i].acceleration, i)
            state.setJointEffort(msg.joints[i].effort, i)

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
            state.setContactPosition_B(name, contact_pos)
            state.setContactVelocity_B(name, contact_vel)
            state.setContactAcceleration_B(name, contact_acc)
            state.setContactWrench_B(name, contact_wrc)