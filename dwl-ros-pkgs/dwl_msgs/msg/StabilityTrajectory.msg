# This message describes the stability trajectory of rigid body system.
#
# A stability trajectory is defined by a sequence of stability states.
# A stability state is defined by the center of mass (CoM) position,
# center of pressure (CoP), or zero moment point (ZMP), position
# and the support polygon for each phase.
# The description of every element is defined in their respective
# messages. This state is described in respective time.
#
# This message consists of a multiple arrays, one for each part of
# of the whole-body state. The goal is to make each of the fields
# optional. For instance, if you floating-base system has a reduced
# number of floating-base joints, i.e. less than 6 dof.
#
# The header specifies the time at which the trajectory is published.


Header header

# The stability trajectory
dwl_msgs/StabilityState[] trajectory