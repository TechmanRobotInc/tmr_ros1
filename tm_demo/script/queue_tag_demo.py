#!/usr/bin/env python

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

"""
Demo: Ask QueueTag

Notice:
1. this demo has robot movement! 
    robot will move to home pose and rotate J3 to about 70 degree
2. start from a new flow so that the Tag state is all 'false'

send 4 points of motion command and 4 QueueTag command in-between,
than ask QueueTag state to check whether the motion command is done

set QueueTag:
using 'tm_driver/set_event' service

ask QueueTag:
using 'tm_driver/ask_sta' service,
and 'sta_response' topic

"""

def queue_tag_demo():
    rospy.init_node('queue_tag_demo')

    # using services
    rospy.wait_for_service('tm_driver/set_event')
    rospy.wait_for_service('tm_driver/set_positions')
    rospy.wait_for_service('tm_driver/ask_sta')

    set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
    set_positions = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)
    ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)

    # 4 points (joint angle[rad])
    points = [
        [0.,0.,0.,0.,0.,0.],
        [0.,0.,0.4,0.,0.,0.],
        [0.,0.,0.8,0.,0.,0.],
        [0.,0.,1.2,0.,0.,0.]
    ]

    # send 4 motion command
    # and set QueueTag command right after motion command (Tag: 1 -> 2 -> 3 -> 4)
    for i in range(4):
        set_positions(SetPositionsRequest.PTP_J, points[i], 0.1, 0.4, 0, False)
        set_event(SetEventRequest.TAG, i + 1, 0)

    # ask sta to check QueueTag state
    i = 0
    while i < 4:
        rospy.sleep(0.2)
        res = ask_sta('01', str(i + 1), 1)
        if res.subcmd == '01':
            data = res.subdata.split(',')
            if data[1] == 'true':
                print("point %d (Tag %s) is reached", i, data[0])
                i = i + 1

if __name__ == '__main__':
    try:
        queue_tag_demo()
    except rospy.ROSInterruptException:
        pass