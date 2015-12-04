# This message describes a whole-body trajectory.
#
# A whole-body trajectory is defined the actual state of the system,
# and a sequence of states.
# The description of every element is defined in their respectives
# messages. This state is described in respective time.
#
# The header specifies the time at which the trajectory is published.


Header header

# The actual whole-body state
dwl_msgs/WholeBodyState actual

# The whole-body trajectory
dwl_msgs/WholeBodyState[] trajectory
