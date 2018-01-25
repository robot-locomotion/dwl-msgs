#!/usr/bin/python
import roslib; roslib.load_manifest('dwl_msgs')

import dwl
from dwl_msgs.msg import WholeBodyState, BaseState, JointState, ContactState
import numpy as np
from xdg.Menu import tmp
from dwl.dwl import FloatingBaseSystem


class WholeBodyStateInterface():
    def __init__(self):
        return
    
    # This method fills the information of a WholeBodyState 
    # ROS message into a WholeBodyState object
    def reset(self, fbs):
        self.is_system = True
        self.fbs = fbs
    
    # Resets the floating-base system information
    def writeFromMessage(self, state, msg):
        # Setting the floating-base system DoF
        num_joints = len(msg.joints)
        num_contacts = len(msg.contacts)
        
        # Setting up the DoF
        state.setJointDoF(num_joints)

        # Getting the time
        state.setTime(msg.time)

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
    
    # This method fills the information of a WholeBodyState object 
    # into a WholeBodyState ROS message
    def writeToMessage(self, msg, state):
        if not is_system_:
            print("Warning: you cannot write the dwl_msg::WholeBodyState "
                "because it wasn't define the FloatingBaseSystem")
            return
        
        # Filling the time information
        msg.time = state.getTime()
        
        # Filling the base state
        msg.base
        for i in range(6):
            base_msg = BaseState()
            base_msg.id = i
            base_msg.position = state.base_pos[i]
            base_msg.velocity = state.base_vel[i]
            base_msg.acceleration = state.base_acc[i]
            msg.base.append(base_msg)
        
        fbs = FloatingBaseSystem()
        # Filling the joint state
        num_joints = state.getJointDoF()
        joint_names = self.fbs.getJointNames()
        for i in range(num_joints):
            joint_msg = JointState()
            joint_msg.name = joint_name[i]
            joint_msg.position = state.getJointPosition(i)
            joint_msg.velocity = state.getJointVelocity(i)
            joint_msg.acceleration = state.getJointAcceleration(i)
            joint_msg.effort = state.getJointEffort(i)
            msg.joint.append(joint_msg)
            
        # Filling the contact state
        for c in state.getContactPosition_B():
            contact_msg = ContactState()
            pos = state.getContactPosition_B(c)
            vel = state.getContactVelocity_B(c)
            acc = state.getContactAcceleration_B(c)
            wrc = state.getContactWrench_B(c)
            contact_msg.position.x = pos[dwl.X]
            contact_msg.position.y = pos[dwl.Y]
            contact_msg.position.z = pos[dwl.Z] 
            contact_msg.velocity.x = vel[dwl.X]
            contact_msg.velocity.y = vel[dwl.Y]
            contact_msg.velocity.z = vel[dwl.Z]
            contact_msg.acceleration.x = acc[dwl.X]
            contact_msg.acceleration.y = acc[dwl.Y]
            contact_msg.acceleration.z = acc[dwl.Z]
            contact_msg.wrench.torque.x = wrch[dwl.AX]
            contact_msg.wrench.torque.y = wrch[dwl.AY]
            contact_msg.wrench.torque.z = wrch[dwl.AZ]
            contact_msg.wrench.force.x = wrch[dwl.LX]
            contact_msg.wrench.force.y = wrch[dwl.LY]
            contact_msg.wrench.force.z = wrch[dwl.LZ]
            msg.contacts[c] = contact_msg