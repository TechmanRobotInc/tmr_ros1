#!/usr/bin/env python

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ': subcmd: %s, subdata: %s\n', data.subcmd, data.subdata)

def ask_sta_demo():
    rospy.init_node('ask_sta_demo')

    rospy.Subscriber('tm_driver/sta_response', StaResponse, callback)

    rospy.wait_for_service('tm_driver/ask_sta')
    ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)

    rospy.sleep(0.5)

    res0 = ask_sta('00', '', 0)
    rospy.loginfo('subcmd: %s, subdata: %s\n', res0.subcmd, res0.subdata)

    rospy.sleep(0.5)

    res1 = ask_sta('00', '', 1)
    rospy.loginfo('subcmd: %s, subdata: %s\n', res1.subcmd, res1.subdata)

    rospy.sleep(0.5)

    res1 = ask_sta('01', '2', 1)
    rospy.loginfo('subcmd: %s, subdata: %s\n', res1.subcmd, res1.subdata)

    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        ask_sta_demo()
    except rospy.ROSInterruptException:
        pass