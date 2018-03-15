import rospy
import roslib; roslib.load_manifest('dwl_msgs')
from dwl_msgs import WholeBodyStateInterface as wb_iface
from dwl_msgs.msg import WholeBodyState


class WholeBodyStatePublisher():
    def __init__(self, topic, fbs):
        # Setting the floating-base system info
        self.fbs = fbs
        self.wb_iface = wb_iface.WholeBodyStateInterface()
        self.wb_iface.reset(self.fbs)

    	# Initializing the publisher
        self.pub = rospy.Publisher(topic, WholeBodyState, queue_size=1)

    def publish(self, state):
        msg = WholeBodyState()
        self.wb_iface.writeToMessage(msg, state)
        self.pub.publish(msg)
        