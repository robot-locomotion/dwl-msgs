# This message describes the stability state of rigid body system.
#
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


# Center of Mass position
geometry_msgs/Vector3 center_of_mass

# Center of Pressure (or Zero Moment Point) position
geometry_msgs/Vector3 center_of_pressure

# Support region formed by the active contacts. Note that are the
# end-effector position in contact with the ground
geometry_msgs/Vector3[] support_region