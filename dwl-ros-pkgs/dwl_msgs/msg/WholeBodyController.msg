# This message describes the whole-body state of controller.
#
# A whole-body controller state is defined by the desired, actual, and
# error of the whole-body state of the system. The description of
# every element is defined in their respective messages.
#
# The header specifies the time at which the trajectory is published.


Header header

# Desired whole-body state
dwl_msgs/WholeBodyState desired

# Actual whole-body state
dwl_msgs/WholeBodyState actual

# Error whole-body state
dwl_msgs/WholeBodyState error

# Command state
dwl_msgs/JointCommand[] command
