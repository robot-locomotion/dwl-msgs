#!/usr/bin/env python
import rospy
import tf
from dwl_msgs.msg import WholeBodyTrajectory, BaseState
from sensor_msgs.msg import JointState


class WholeBodyTrajectoryPublisher():
    def __init__(self):
        # Defining the subscriber
        rospy.Subscriber("/hyq/plan", WholeBodyTrajectory, self.callback)

        # Defining the base state publisher (tf broadcaster)
        self.tf_pub = tf.TransformBroadcaster()

        # Defining the joint state publisher
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=5)


    def callback(self, msg):
        # Setting the current state
        state = msg.actual
        
        # Publishing the current joint state
        self.publishBaseTf(state)
        self.publishJointState(state)
        
        # Publishing the joint state trajectory
        for i in range(len(msg.trajectory)):
            duration = msg.trajectory[i].time - state.time
            rospy.sleep(duration)
            state = msg.trajectory[i]
            self.publishBaseTf(state)
            self.publishJointState(state)

       
    def publishBaseTf(self, state):
        self.tf_pub.sendTransform((state.base[BaseState.LX].position,
                                   state.base[BaseState.LY].position,
                                   state.base[BaseState.LZ].position),
                                  tf.transformations.quaternion_from_euler(state.base[BaseState.AX].position,
                                                                           state.base[BaseState.AY].position,
                                                                           state.base[BaseState.AZ].position,
                                                                           'rzyz'),
                                  rospy.Time.now(),
                                  "base_link",
                                  "world")        
        
        
    def publishJointState(self, state):
        # Getting the base and joint messages
        joint_msg = JointState()
        
        # Setting up the time
        joint_msg.header.stamp = rospy.Time.now()

        # Setting up the floating-base system DoF
        num_base_joints = len(state.base)
        num_joints = (num_base_joints + len(state.joints))
        
        # Initializing the joint state sizes
        joint_msg.name = num_joints * [""]
        joint_msg.position = num_joints * [0.0]
        joint_msg.velocity = num_joints * [0.0]
        joint_msg.effort = num_joints * [0.0]
        
        # Setting the whole-body state message
        for i in range(num_joints):
            if i < num_base_joints:
                joint_msg.name[i] = state.base[i].name
                joint_msg.position[i] = state.base[i].position
                joint_msg.velocity[i] = state.base[i].velocity
                joint_msg.effort[i] = 0.0
            else:
                joint_idx = i - num_base_joints
                joint_msg.name[i] = state.joints[joint_idx].name
                joint_msg.position[i] = state.joints[joint_idx].position;
                joint_msg.velocity[i] = state.joints[joint_idx].velocity;
                joint_msg.effort[i] = state.joints[joint_idx].effort;
                
        # Publishing the current joint state       
        self.joint_pub.publish(joint_msg)
        


if __name__ == '__main__':
    rospy.init_node('whole_body_trajectory_publisher')
        
    jsp = WholeBodyTrajectoryPublisher()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
