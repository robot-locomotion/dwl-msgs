#!/usr/bin/python
import roslib; roslib.load_manifest('dwl_msgs')

import dwl
from dwl_msgs.msg import WholeBodyState
from dwl_msgs import WholeBodyStateInterface as interface
import numpy as np
import rosbag



def extractWholeBodyState(bagfile, topic, initial_time = 0., duration = 0.):
    print('Extracting the WholeBodyState message in topic' + topic +
          ' from rosbag ' + bagfile)

    n = 0
    with rosbag.Bag(bagfile, 'r') as bag:
        starting_time = bag.get_start_time() + initial_time
        if (duration == 0.):
            ending_time = bag.get_end_time()
            duration = ending_time - starting_time
        else:
            ending_time = starting_time + duration
        
        # Recording the whole-body state trajectory
        ws_list = []
        for (topic, msg, ts) in bag.read_messages(topics=str(topic)):
            # Recording the data in the desired time range
            if (ts.to_sec() >= starting_time and ts.to_sec() <= ending_time):
                # Getting the current time
                if (n == 0):
                    initial_time = ts.to_sec()
                time = ts.to_sec() - initial_time

                # Construct an instance of the WholeBodyState class, which wraps the C++ class.
                ws = dwl.WholeBodyState()
                
                wbi = interface.WholeBodyStateInterface()
                wbi.writeFromMessage(ws, msg)
                
                # Defining our internal time rather than CPU time
                ws.setTime(time)

                # Appeding the whole-state to the list
                ws_list.append(ws)

                n += 1
    print('Loaded ' + str(n) + ' whole-body controller states from the bagfile.')
    
    return ws_list



def extractWholeBodyControllerState(bagfile, topic):
    print('Extracting the WholeBodyControllerState message in topic' + topic +
          ' from rosbag ' + bagfile)

    n = 0
    initial_time = 0
    with rosbag.Bag(bagfile, 'r') as bag:
        initial_time = 0.
        duration = 0.
        starting_time = bag.get_start_time() + initial_time
        if (duration == 0.0):
            ending_time = bag.get_end_time()
            duration = ending_time - starting_time
        else:
            ending_time = starting_time + duration
        
        # Recording the whole-body state trajectory
        actual_ws_list = []
        desired_ws_list = []
        error_ws_list = []
        for (topic, msg, ts) in bag.read_messages(topics=str(topic)):
            # Recording the data in the desired time range
            if (ts.to_sec() >= starting_time and ts.to_sec() <= ending_time):
                # Getting the current time
                if (n == 0):
                    initial_time = ts.to_sec()
                time = ts.to_sec() - initial_time

                # Defined the desired, actual and error whole-body states
                actual_ws = dwl.WholeBodyState()
                desired_ws = dwl.WholeBodyState()
                error_ws = dwl.WholeBodyState()
                
                wbi = interface.WholeBodyStateInterface()
                wbi.writeFromMessage(actual_ws, msg.actual)
                wbi.writeFromMessage(desired_ws, msg.desired)
                wbi.writeFromMessage(error_ws, msg.error)

                # Defining our internal time rather than CPU time
                actual_ws.setTime(time)
                desired_ws.setTime(time)
                error_ws.setTime(time)
                
                # Appeding the whole-state to the list
                actual_ws_list.append(actual_ws)
                desired_ws_list.append(desired_ws)
                error_ws_list.append(error_ws)

                n += 1
    print('Loaded ' + str(n) + ' whole-body controller states from the bagfile.')
    
    return actual_ws_list, desired_ws_list, error_ws_list