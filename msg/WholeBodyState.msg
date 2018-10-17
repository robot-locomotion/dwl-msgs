# This message describes the whole-body state of rigid body system.
#
# A whole-body state is defined by the base, joints and contact states.
# The description of every element is defined in their respective
# messages. This state is described in respective time.
#
# This message consists of a multiple arrays, one for each part of
# of the whole-body state. The goal is to make each of the fields
# optional. For instance, if you floating-base system has a reduced
# number of floating-base joints, i.e. less than 6 dof.
#
# The header specifies the time at which the state is published.


Header header

# This represent the time state
float64 time

# This represents the base state (position, velocity, acceleration and effort)
dwl_msgs/BaseState base

# This represents the joint state (position, velocity, acceleration and effort)
dwl_msgs/JointState[] joints

# This represents the end-effector state (cartesian position and contact forces)
dwl_msgs/ContactState[] contacts
