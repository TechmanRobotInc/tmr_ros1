#!/usr/bin/env python

import rospy
#from tmr_msgs.srv import *

import os
import shutil
import sys

from _modify_urdf import *

def _gen_urdf():
    rospy.init_node('modify_urdf')

    if len(sys.argv) < 3:
        rospy.logwarn('usage: modify_urdf_node model new_model replace')
        return

    model = sys.argv[1]
    new_model = sys.argv[2]
    replace = False
    if len(sys.argv) == 4:
        if sys.argv[3].upper() == 'REPLACE':
            replace = True
            rospy.logwarn('origin urdf file will be replaced')


    rospy.wait_for_service('tmr/ask_item')
    ask_item = rospy.ServiceProxy('tmr/ask_item', AskItem)
    res_dh = ask_item('dh', 'DHTable', 1.0)
    res_dd = ask_item('dd', 'DeltaDH', 1.0)

    if not res_dh.value.startswith('DHTable={') or not res_dh.value.endswith('}'):
        rospy.logerr('invalid dh')
        return
    if not res_dd.value.startswith('DeltaDH={') or not res_dd.value.endswith('}'):
        rospy.logerr('invalid delta_dh')
        return

    rospy.loginfo(res_dh.value)
    rospy.loginfo(res_dd.value)

    dh_strs = res_dh.value[9:-1].split(',')
    dd_strs = res_dd.value[9:-1].split(',')
    '''
    res_dh = 'DHTable={0,-90,0,145.1,0,-277,277,-90,0,429,0,0,-187,187,0,0,411.5,0,0,-162,162,90,90,0,-122.2,0,-187,187,0,90,0,106,0,-187,187,0,0,0,114.4,0,-277,277}'
    res_dd = 'DeltaDH={-0.001059821,0.02508766,0.009534874,0,0.001116668,0.06614932,0.308224,0.0287381,0.06797475,-0.0319523,0.3752921,0.06614756,-0.006998898,0.06792655,-0.06083903,0.02092069,0.02965812,-0.1331249,0.06793034,0.02077797,0.08265772,0.03200645,0.01835932,0.06145732,0.08273286,0.6686108,0.6972408,-0.1793097,-0.0794057,1.425708}'
    rospy.loginfo(res_dh)
    rospy.loginfo(res_dd)
    dh_strs = res_dh[9:-1].split(',')
    dd_strs = res_dd[9:-1].split(',')
    '''

    if len(dh_strs) != 42:
        rospy.logerr('invalid dh')
        return
    if len(dd_strs) != 30:
        rospy.logerr('invalid delta_dh')
        return

    dh = [float(i) for i in dh_strs]
    dd = [float(i) for i in dd_strs]

    # find urdf path
    curr_path = os.path.dirname(os.path.abspath(__file__))
    dirs = ['src', 'devel']
    ind = -1
    for d in dirs:
        ind = curr_path.find(d)
        if (ind != -1):
            break
    if (ind == -1) :
        rospy.logerr('can not find workspace directory')
        return
    src_path = curr_path[:ind] + 'src'
    urdf_path = ''
    for dirpath, dirnames, filenames in os.walk(src_path):
        if dirpath.endswith('tmr_description'):
            urdf_path = dirpath + '/urdf'
            break
    if (urdf_path == ''):
        rospy.logerr('can not find urdf directory')
        return

    urdf_name = '/' + model + '.urdf'
    new_urdf_name = '/' + new_model + '.urdf'

    file_in = urdf_path + urdf_name
    file_out = urdf_path + new_urdf_name

    rospy.loginfo(file_in)

    fr = open(file_in, 'r')
    link_data = fr.read()
    fr.close()

    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh)

    link_data = ET.tostring(root, encoding='UTF-8').decode('UTF-8')

    file_save = ''
    if replace:
        file_save = file_in
        rospy.loginfo('copy and rename origin urdf file')
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    rospy.loginfo(file_save)

    fw = open(file_save, 'w')
    fw.write(link_data)
    fw.close()

def main():
    try:
        _gen_urdf()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
