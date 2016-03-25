#!/usr/bin/env python
import rospy
import rosbag
import message_filters
from datetime import datetime
from dwl_msgs.msg import WholeBodyTrajectory, ReducedTrajectory
from std_msgs.msg import Bool


class WholeBodyTrajectoryRecorder():
    def __init__(self):
        # Defining the subscribers
        self.full_sub = message_filters.Subscriber('/hyq/plan', WholeBodyTrajectory)
        self.reduced_sub = message_filters.Subscriber('/hyq/reduced_plan', ReducedTrajectory)
        
        # Setting up the time synchronization
        ts = message_filters.ApproximateTimeSynchronizer([self.full_sub, self.reduced_sub], 10, 0.1)
        
        # Registering just one callback function
        ts.registerCallback(self.callback)


    def callback(self, full_plan, reduced_plan):
        # Getting the current date time
        i = datetime.now()
        
        # Defining the recorder
        bag = rosbag.Bag(i.strftime('%Y-%m-%d-%H-%M-%S') + ".bag", 'w')
        
        try:
            activate = Bool()
            activate.data = True
            bag.write('playing', activate)

            bag.write('/hyq/plan', full_plan)
            bag.write('/hyq/reduced_plan', reduced_plan)
        finally:
            bag.close()



if __name__ == '__main__':
    rospy.init_node('whole_body_trajectory_recorder')
        
    jsp = WholeBodyTrajectoryRecorder()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
