# This message describes the states of a floating-base joint.
#
# A floating-base joint describes the traslational (LX, LY, or LZ)
# and angular (AX, AY, or AZ) displacement w.r.t to the world frame.
#
# The state of a floating-base joint (revolute or prismatic) is
# defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the acceleration of the joint (rad/s^2 or m/s^2)
#
# There are 6 type of floating-base joint which is identified by
# its id, which keeps the spatial algebra convention, please see all
# the below identifiers. Additionally, a floating-base joint could be
# identified with its name


# Floating-base identifiers
uint8 AX=0
uint8 AY=1
uint8 AZ=2
uint8 LX=3
uint8 LY=4
uint8 LZ=5

# Name and id of the floating-base joint
string name
uint8 id

# States of the floating-base joint
float64 position
float64 velocity
float64 acceleration
