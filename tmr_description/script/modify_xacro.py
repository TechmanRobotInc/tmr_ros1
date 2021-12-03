#!/usr/bin/env python

import rospy
from tm_msgs.srv import *

import os
import shutil
import sys

from _modify_urdf import *

def _gen_xarco():
    rospy.init_node('modify_xacro')

    if len(sys.argv) < 3:
        rospy.logwarn('usage: modify_xacro_node model new_model replace')
        return

    model = sys.argv[1]
    new_model = sys.argv[2]
    replace = False
    if len(sys.argv) == 4:
        if sys.argv[3].upper() == 'REPLACE':
            replace = True
            rospy.logwarn('origin xacro file will be replaced')


    rospy.wait_for_service('tm/ask_item')
    ask_item = rospy.ServiceProxy('tm/ask_item', AskItem)
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

    # find xacro path
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
    xacro_path = ''
    for dirpath, dirnames, filenames in os.walk(src_path):
        if dirpath.endswith('tmr_description'):
            xacro_path = dirpath + '/xacro'
            break
    if (xacro_path == ''):
        rospy.logerr('can not find xacro directory')
        return

    xacro_name = '/macro.' + model + '.urdf.xacro'
    new_xacro_name = '/macro.' + new_model + '.urdf.xacro'

    file_in = xacro_path + xacro_name
    file_out = xacro_path + new_xacro_name

    link_tag = '<!--LinkDescription-->'
    link_head = '<?xml version=\'1.0\' encoding=\'UTF-8\'?>\n'
    link_start = '<data xmlns:xacro="http://wiki.ros.org/xacro">'
    link_end = '</data>'

    rospy.loginfo(file_in)

    fr = open(file_in, 'r')
    data_in = fr.read()
    fr.close()
    datas = data_in.split(link_tag)

    if len(datas) < 3:
        rospy.logerr('invalid tm...xacro')
        return

    link_data = link_start + datas[1] + link_end
    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh, '${prefix}')

    link_data = ET.tostring(root, encoding='UTF-8').decode('UTF-8')
    link_data = link_data.replace('ns0', 'xacro')
    link_data = link_data.replace(link_head, '', 1)
    link_data = link_data.replace(link_start, link_tag, 1)
    link_data = link_data.replace(link_end, link_tag, 1)

    data_out = datas[0] + link_data + datas[2]

    file_save = ''
    if replace:
        file_save = file_in
        rospy.loginfo('copy and rename origin xacro file')
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    rospy.loginfo(file_save)

    fw = open(file_save, 'w')
    fw.write(data_out)
    fw.close()

def main():
    try:
        _gen_xarco()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
