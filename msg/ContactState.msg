# This message describes the state of contact or end-effector body.
#
# The contact states are defined w.r.t the floating-base frame. A
# contact state is defined by:
#  * the position of the contact (m),
#  * the velocity of the contact (m/s),
#  * the acceleration of the contact (m/s^2), and
#  * the wrench of the contact (N or Nm)
#
# Each joint is uniquely identified by its name
# the below identifiers. Additionally, a floating-base joint could be


# Name of the contact body
string name

# State of the contact body
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Twist acceleration
geometry_msgs/Wrench wrench