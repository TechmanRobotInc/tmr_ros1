from datetime import datetime

import os
import queue
import signal
import threading

import cv2

from cv_bridge import CvBridge
from flask import Flask, jsonify, request

import numpy as np
import rospy

from sensor_msgs.msg import Image
from waitress import serve


def signal_handler(signum, frame):
    global leaveThread
    print('signal_handler: capture signal ' + str(signum))
    leaveThread = True
    this_process = os.getpgid(0)
    os.killpg(this_process, signal.SIGKILL)


signal.signal(signal.SIGINT, signal_handler)
conditionVar = threading.Condition()
imageQ = queue.Queue()
leaveThread = False


def set_image_and_notify_send(img):
    global conditionVar
    global imageQ
    conditionVar.acquire()
    imageQ.put(img)
    conditionVar.notify()
    conditionVar.release()


def image_publisher(image, imagepub):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(image)
    imagepub.publish(msg)


def pub_data_thread(imagepub):
    global conditionVar
    global imageQ
    conditionVar.acquire()
    while(True):
        conditionVar.wait()
        while(not imageQ.empty()):

            file2np = np.fromstring(imageQ.get(), np.uint8)
            img = cv2.imdecode(file2np, cv2.IMREAD_UNCHANGED)
            image_publisher(img, imagepub)
        if(leaveThread):
            break
    conditionVar.release()


def fake_result(m_method):
    # clsssification
    if m_method == 'CLS':
        # inference img here
        result = {
            'message': 'success',
            'result': 'NG',
            'score': 0.987
        }
    # detection
    elif m_method == 'DET':
        # inference img here
        result = {
            'message': 'success',
            'annotations':
            [
                {
                    'box_cx': 150,
                    'box_cy': 150,
                    'box_w': 100,
                    'box_h': 100,
                    'label': 'apple',
                    'score': 0.964,
                    'rotate': -45
                },
                {
                    'box_cx': 550,
                    'box_cy': 550,
                    'box_w': 100,
                    'box_h': 100,
                    'label': 'car',
                    'score': 1.000,
                    'rotation': 0
                },
                {
                    'box_cx': 350,
                    'box_cy': 350,
                    'box_w': 150,
                    'box_h': 150,
                    'label': 'mobilephone',
                    'score': 0.886,
                    'rotation': 135
                }
            ],
            'result': None
        }
    # no method
    else:
        result = {
            'message': 'no method',
            'result': None
        }
    return result


def get_none():
    print('\n[{0}] [{1}] -> Get()'.format(request.environ['REMOTE_ADDR'], datetime.now()))
    # user defined method
    result = {
        'result': 'api',
        'message': 'running',
    }
    return jsonify(result)


def get(m_method):
    print('\n[{0}] [{1}] -> Get({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))
    # user defined method
    if m_method == 'status':
        result = {
            'result': 'status',
            'message': 'im ok'
        }
    else:
        result = {
            'result': 'fail',
            'message': 'wrong request'
        }
    return jsonify(result)


def post(m_method):
    print('\n[{0}] [{1}] -> Post({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))
    # get key/value
    model_id = request.args.get('model_id')
    print('model_id: {}'.format(model_id))

    # check key/value
    if model_id is None:
        print('model_id is not set')
        result = {
            'message': 'fail',
            'result': 'model_id required'
        }
        return jsonify(result)

    set_image_and_notify_send(request.files['file'].read())
    result = fake_result(m_method)

    return jsonify(result)


def set_route(app):
    app.route('/api/<string:m_method>', methods=['POST'])(post)
    app.route('/api/<string:m_method>', methods=['GET'])(get)
    app.route('/api', methods=['GET'])(get_none)


def get_image():
    rospy.init_node('get_image')
    imagepub = rospy.Publisher('techman_image', Image, queue_size=10)
    t = threading.Thread(target=pub_data_thread, args=(imagepub,))
    t.start()
    app = Flask(__name__)
    set_route(app)
    print('Listening on an IP port:6189 combination')
    serve(app, port=6189)
    print('end serve')


if __name__ == '__main__':
    try:
        get_image()
    except rospy.ROSInterruptException:
        pass
