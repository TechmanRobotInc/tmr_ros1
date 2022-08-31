import sys
import socket
import queue
import signal
import rospy
from sensor_msgs.msg import Image 

from flask import Flask, request, jsonify
import numpy as np
import cv2
from waitress import serve
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
import threading
import os
import signal

def signal_handler(signum, frame):
    print('signal_handler: caught signal ' + str(signum))
    leaveThread = True
    this_process = os.getpgid(0)
    os.killpg(this_process, signal.SIGKILL)
signal.signal(signal.SIGINT, signal_handler)
conditionVar = threading.Condition()
imageQ = queue.Queue()
leaveThread = False
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
            image_publisher(img,imagepub)
        if(leaveThread):
            break
    conditionVar.release()
    
def set_image_and_notify_send(img):
    global conditionVar
    global imageQ
    conditionVar.acquire()
    imageQ.put(img)
    conditionVar.notify()
    conditionVar.release()
def post(m_method):      
    print('\n[{0}] [{1}] -> Post({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))          
    # get key/value
    model_id = request.args.get('model_id')
    print('model_id: {}'.format(model_id))

    # check key/value
    if model_id is None:
        print("model_id is not set")    
        result={                    
            "message": "fail",
            "result": "model_id required"
        }  
        return jsonify(result)

    # convert image data        
    #file2np = np.fromstring(request.files['file'].read(), np.uint8)        
    #img = cv2.imdecode(file2np, cv2.IMREAD_UNCHANGED)
    #cv2.imwrite('test.png',img)

    set_image_and_notify_send(request.files['file'].read())
    result = self.fake_result(m_method)    

    return jsonify(result)

def get(self,m_method):
    print('\n[{0}] [{1}] -> Get({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))
    # user defined method
    if m_method == 'status':
        result = {
            "result": "status",
            "message": "im ok"
        }
    else:
        result = {
            "result": "fail",
            "message": "wrong request"            
        }
    return jsonify(result)
def get_none():    
        print('\n[{0}] [{1}] -> Get()'.format(request.environ['REMOTE_ADDR'], datetime.now()))
        # user defined method
        result = {
            "result": "api",
            "message": "running",
        } 
        return jsonify(result)        
def set_route(app):
    app.route('/api/<string:m_method>', methods=['POST'])(post)
    app.route('/api/<string:m_method>', methods=['GET'])(get)
    app.route('/api', methods=['GET'])(get_none)
    
def get_image():
    rospy.init_node('ask_item_demo')
    imagepub = rospy.Publisher('techman_image', Image, queue_size=10)
    t = threading.Thread(target = pub_data_thread, args=(imagepub,))
    t.start()
    print("a")
    app = Flask(__name__)
    print("b")
    set_route(app)
    print("c")
    print("Listening on an ip port:6189 combination")
    serve(app, port=6189)
    print("end serve")

if __name__ == '__main__':
    try:
        get_image()
    except rospy.ROSInterruptException:
        pass
