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

send 4 points of motion command and 4 QueueTag command in-between.
Two way to check whether the motion command is done:
1. Listening to 'tm_driver/sta_response' topic.
2. Polling QueueTag state by 'tm_driver/ask_sta' service.


set QueueTag:
using 'tm_driver/set_event' service

ask QueueTag:
using 'tm_driver/ask_sta' service,
and 'tm_driver/sta_response' topic

"""

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + ': %s', msg.subdata)
    if msg.subcmd == '01':
        data = msg.subdata.split(',')
        if data[1] == 'true':
            rospy.loginfo('point (Tag %s) is reached', data[0])

def queue_tag_demo(by_polling):
    rospy.init_node('queue_tag_demo')

    if not by_polling:
        # listen to 'tm_driver/sta_response' topic
        rospy.Subscriber('tm_driver/sta_response', StaResponse, callback)

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

    if by_polling:
        # ask sta to check QueueTag state
        i = 0
        while i < 4:
            rospy.sleep(0.2)
            res = ask_sta('01', str(i + 1), 1)
            if res.subcmd == '01':
                data = res.subdata.split(',')
                if data[1] == 'true':
                    rospy.loginfo('point %d (Tag %s) is reached', i + 1, data[0])
                    i = i + 1
    else:
        rospy.spin()

if __name__ == '__main__':
    try:
        #queue_tag_demo(False)
        queue_tag_demo(True)
    except rospy.ROSInterruptException:
        pass
