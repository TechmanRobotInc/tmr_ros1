#!/usr/bin/env python

"""
Demo: Ask item (HandCamera_Value, Delta)
'ask_item' service send 'Read' request command to controller.
'tm_driver' node receive the result and publish it to 'tm_driver/svr_response'.
('tm_driver' node must be running)

'ask_item' service request param.:
string id -> response 'id' is same as request 'id'
string item -> item_name you want to ask
float64 wait_time ->

If 'wait_time' == 0,
the service call is non-blocking, only send 'Read' request.
The response data is NULL,
but you can still get result from topic 'tm_driver/svr_response'.

If 'wait_time' > 0,
the service call is blocking with timeout 'wait_time' sec. until the result is received.
You can get result in response data.
"""

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ': id: %s, content: %s\n', data.id, data.content)

def ask_item_demo():
    rospy.init_node('ask_item_demo')

    # listen to 'tm_driver/svr_response' topic
    rospy.Subscriber('tm_driver/svr_response', SvrResponse, callback)

    # using 'tm_driver/ask_item' service
    rospy.wait_for_service('tm_driver/ask_item')
    ask_item = rospy.ServiceProxy('tm_driver/ask_item', AskItem)

    rospy.sleep(0.5)

    # ask hand-eye info. (non-block)
    res0 = ask_item('he0', 'HandCamera_Value', 0)
    rospy.loginfo('id: %s, value: %s\n', res0.id, res0.value)

    """
    Note
    HandCamera_Value string format:

    for example:
    "HandCamera_Value={0.36,78.39,43.49,0.68,-1.58,-178.04}"

    { X,Y,Z,R,P,Y } unit: mm | deg

    transformation from camera frame to tool flange
    tool0
         T
          camera

    """

    rospy.sleep(0.5)

    # ask hand-eye info. (block until receive or timeout)
    res1 = ask_item('he1', 'HandCamera_Value', 1)
    rospy.loginfo('id: %s, value: %s\n', res1.id, res1.value)

    rospy.sleep(0.5)

    # ask delta DH info.
    resd = ask_item('dd', 'DeltaDH', 1)
    rospy.loginfo('id: %s, value: %s\n', resd.id, resd.value)

    """
    Note
    DeltaDH string format:

    for example:
    "DeltaDH={-0.001059821,0.02508766,0.009534874,0,0.001116668,0.06614932,0.308224,0.0287381,0.06797475,-0.0319523,0.3752921,0.06614756,-0.006998898,0.06792655,-0.06083903,0.02092069,0.02965812,-0.1331249,0.06793034,0.02077797,0.08265772,0.03200645,0.01835932,0.06145732,0.08273286,0.6686108,0.6972408,-0.1793097,-0.0794057,1.425708}"
 
    { d_theta1, d_alpha1, d_a1, d_d1, d_beta1, d_theta2, .... }  unit: mm | deg
    (prefix: d -> delta)

    theta = theta + d_theta
    alpha = alpha + d_alpha
    beta = d_beta
    a = a + d_a
    d = d + d_d
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    T = [
        cb * ct - sb * sa*st, -ca*st, sb * ct + cb * sa*st, ct * a
        cb * st + sb * sa*ct,  ca*ct, sb * st - cb * sa*ct, st * a
                 -sb * ca,     sa,              cb * ca,         d
    ]
    """

    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        ask_item_demo()
    except rospy.ROSInterruptException:
        pass
