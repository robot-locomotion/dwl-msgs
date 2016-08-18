# This message describes the reduced-body state of rigid body system.
#
# A reduced state is defined by the center of mass (CoM) position,
# center of pressure (CoP), or zero moment point (ZMP), position
# and the support polygon for each phase.
# The description of every element is defined in their respective
# messages. This state is described in respective time.


# This represent the time state
float64 time

# Center of Mass position
geometry_msgs/Vector3 center_of_mass

# Center of Pressure (or Zero Moment Point) position
geometry_msgs/Vector3 center_of_pressure

# Support region formed by the active contacts. Note that are the
# end-effector position in contact with the ground
geometry_msgs/Vector3[] support_region
