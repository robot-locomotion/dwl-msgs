# This message describes the reduced-body trajectory of rigid body system.
#
# A reduced trajectory is defined by a sequence of reduced states.
# A reduced state is defined by the center of mass (CoM) position,
# center of pressure (CoP), or zero moment point (ZMP), position
# and the support polygon for each phase.
# The description of every element is defined in their respective
# messages.
#
# This message consists of a multiple arrays, one for each part of
# of the reduced-body state.
#
# The header specifies the time at which the trajectory is published.


Header header

# The actual reduced trajectory
dwl_msgs/ReducedBodyState actual

# The reduced trajectory
dwl_msgs/ReducedBodyState[] trajectory
