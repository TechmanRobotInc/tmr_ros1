from __future__ import print_function
import sys
import math
import numpy as np

# from xml.etree import ElementTree
import xml.etree.cElementTree as ET

#  always print floating point numbers using fixed point notation
np.set_printoptions(suppress=True)

_DoF = 6
_A     = 0
_ALPHA = 1
_BETA  = 2
_D     = 3
_THETA = 4
_LLIM  = 5
_ULIM  = 6

# theta-alpha-a-d-t-l-u, unit mm | deg
#tm_DH = [0.,-90.,0.,145.2,0.,-270.,270,-90.,0.,429.,0.,0.,-180.,180.,0.,0.,411.5,0.,0.,-158.,158.,90.,90.,0.,-122.3,0.,-180.,180.,0.,90.,0.,106.,0.,-180.,180.,0.,0.,0.,113.15,0.,-270.,270.]
#'DHTable={0.,-90.,0.,145.2,0.,-270.,270,-90.,0.,429.,0.,0.,-180.,180.,0.,0.,411.5,0.,0.,-158.,158.,90.,90.,0.,-122.3,0.,-180.,180.,0.,90.,0.,106.,0.,-180.,180.,0.,0.,0.,113.15,0.,-270.,270.}'
# theta-alpha-a-d-beta, unit mm | deg
#tm_DeltaDH = [-0.001059821,0.02508766,0.009534874,0.,0.001116668,0.06614932,0.308224,0.0287381,0.06797475,-0.0319523,0.3752921,0.06614756,-0.006998898,0.06792655,-0.06083903,0.02092069,0.02965812,-0.1331249,0.06793034,0.02077797,0.08265772,0.03200645,0.01835932,0.06145732,0.08273286,0.6686108,0.6972408,-0.1793097,-0.0794057,1.425708]
#'DeltaDH={-0.001059821,0.02508766,0.009534874,0.,0.001116668,0.06614932,0.308224,0.0287381,0.06797475,-0.0319523,0.3752921,0.06614756,-0.006998898,0.06792655,-0.06083903,0.02092069,0.02965812,-0.1331249,0.06793034,0.02077797,0.08265772,0.03200645,0.01835932,0.06145732,0.08273286,0.6686108,0.6972408,-0.1793097,-0.0794057,1.425708}'
#tm_DeltaDH = [0.] * 30

# Checks if a matrix is a valid rotation matrix.
def is_rotation_matrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rot_x(x):
    return np.array([[1,           0,            0],
                     [0, math.cos(x), -math.sin(x)],
                     [0, math.sin(x),  math.cos(x)]])
def rot_y(y):
    return np.array([[ math.cos(y), 0, math.sin(y)],
                     [           0, 1,           0],
                     [-math.sin(y), 0, math.cos(y)]])
def rot_z(z):
    return np.array([[math.cos(z), -math.sin(z), 0],
                     [math.sin(z),  math.cos(z), 0],
                     [          0,            0, 1]])

def T_a_alpha(a, alpha):
    return np.array([[1,               0,                0, a],
                     [0, math.cos(alpha), -math.sin(alpha), 0],
                     [0, math.sin(alpha),  math.cos(alpha), 0],
                     [0,               0,                0, 1]])
def T_beta(beta):
    return np.array([[ math.cos(beta), 0, math.sin(beta), 0],
                     [              0, 1,              0, 0],
                     [-math.sin(beta), 0, math.cos(beta), 0],
                     [              0, 0,              0, 1]])
def T_d_theta(d, theta):
    return np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                     [math.sin(theta),  math.cos(theta), 0, 0],
                     [              0,                0, 1, d],
                     [              0,                0, 0, 1]])

# Calculates Rotation Matrix given euler angles.
def rotation_matrix_from_euler_angles(theta):
    R_x = rot_x(theta[0])
    R_y = rot_y(theta[1])
    R_z = rot_z(theta[2])
    return np.dot(R_z, np.dot(R_y, R_x))
    #return R_z @ R_y @ R_x

# Calculates rotation matrix to euler angles
def euler_angles_from_rotation_matrix(R) :
    assert(is_rotation_matrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# URDF DH ((5+2) x 6) from TM DH Table (7x6) and Delta DH (5x6)
# a-alpha-beta-d-theta <-- theta-alpha-a-d-t-l-u + delta(theta-alpha-a-d-beta)
def urdf_DH_from_tm_DH(tm_DH, tm_DeltaDH):
    assert(len(tm_DH) == 7*_DoF and len(tm_DeltaDH) == 5*_DoF)

    urdf_DH = np.zeros([_DoF + 1, 7])
    # urdf_DH[0, _A    ] = 0.
    # urdf_DH[0, _ALPHA] = 0.
    # urdf_DH[0, _BETA ] = 0.
    for i in range(_DoF):
        urdf_DH[i    , _D    ] =     0.001 * (tm_DH[7*i + 3] + tm_DeltaDH[5*i + 3])
        urdf_DH[i    , _THETA] = math.radians(tm_DH[7*i + 0] + tm_DeltaDH[5*i + 0])
        urdf_DH[i    , _LLIM ] = math.radians(tm_DH[7*i + 5])
        urdf_DH[i    , _ULIM ] = math.radians(tm_DH[7*i + 6])
        urdf_DH[i + 1, _A    ] =     0.001 * (tm_DH[7*i + 2] + tm_DeltaDH[5*i + 2])
        urdf_DH[i + 1, _ALPHA] = math.radians(tm_DH[7*i + 1] + tm_DeltaDH[5*i + 1])
        urdf_DH[i + 1, _BETA ] = math.radians(tm_DeltaDH[5*i + 4])
    # urdf_DH[_DoF, _D    ] = 0.
    # urdf_DH[_DoF, _THETA] = 0.
    return urdf_DH

def xyzrpys_from_urdf_DH(udh):
    np.set_printoptions(suppress=True)
    xyzs = np.zeros([_DoF + 1, 3])
    rpys = np.zeros([_DoF + 1, 3])
    for i in range(_DoF + 1):
        Ta = T_a_alpha(udh[i, _A], udh[i, _ALPHA])
        Tb = T_beta(udh[i, _BETA])
        Tc = T_d_theta(udh[i, _D], udh[i, _THETA])
        T = np.dot(Ta, np.dot(Tb, Tc))
        #T =  Ta @ Tb @ Tc
        #R = T[0:3, 0:3]
        xyzs[i] = T[0:3, 3]
        rpys[i] = euler_angles_from_rotation_matrix(T[0:3, 0:3])

        print('link', i+1, ':')
        print('xyz :', np.round(xyzs[i], 6))
        print('rpy :', np.round(rpys[i], 6))
        print('T :\n', np.round(T, 6))
        print('\n')
    return xyzs, rpys

def str_from_nparray(nparray):
    string = ''
    for value in nparray:
        #string += str(value)
        string += '{:f}'.format(value)
        string += ' '
        
    string = string[:-1]
    return string

def pretty_xml(element, indent, newline, level = 0):
    if element:
        if element.text == None or element.text.isspace():
            element.text = newline + indent * (level + 1)      
        else:    
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)    

    temp = list(element)
    for subelement in temp:    
        if temp.index(subelement) < (len(temp) - 1):
            subelement.tail = newline + indent * (level + 1)    
        else:
            subelement.tail = newline + indent * level    
        pretty_xml(subelement, indent, newline, level = level + 1)

def urdf_generator(file, xyzs, rpys, udh):
    #tree = ET.parse(file)
    link_tag = '<!--LinkDescription-->'
    link_head = '<?xml version=\'1.0\' encoding=\'UTF-8\'?>\n'
    link_start = '<data xmlns:xacro="http://wiki.ros.org/xacro">'
    link_end = '</data>'
    f = open(file, 'r')
    read_data = f.read()
    data = read_data.split(link_tag)
    link_data = link_start + data[1] + link_end
    #print(link_data)
    link_root = ET.fromstring(link_data)

    #root = ET.fromstring(read_data)
    #root = tree.getroot()
    #for elem in tree.iter(tag = 'joint'):
    for elem in link_root.findall('joint'):
        for index in elem.attrib:
            if index == 'name' and elem.attrib[index] == '${prefix}base_fixed_joint':
                origin = elem.find('origin')
                origin.attrib['xyz'] = '0.0 0.0 0.0'
                origin.attrib['rpy'] = '0.0 0.0 0.0'

            elif index == 'name' and elem.attrib[index] == '${prefix}joint_1':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[0, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[0, :], 8))
                limit = elem.find('limit')
                limit.attrib['lower'] = str(np.round(udh[0, _LLIM], 4))
                limit.attrib['upper'] = str(np.round(udh[0, _ULIM], 4))

            elif index == 'name' and elem.attrib[index] == '${prefix}joint_2':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[1, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[1, :], 8))
                limit = elem.find('limit')
                limit.attrib['lower'] = str(np.round(udh[1, _LLIM], 4))
                limit.attrib['upper'] = str(np.round(udh[1, _ULIM], 4))
 
            elif index == 'name' and elem.attrib[index] == '${prefix}joint_3':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[2, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[2, :], 8))
                limit = elem.find('limit')
                limit.attrib['lower'] = str(np.round(udh[2, _LLIM], 4))
                limit.attrib['upper'] = str(np.round(udh[2, _ULIM], 4))

            elif index == 'name' and elem.attrib[index] == '${prefix}joint_4':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[3, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[3, :], 8))
                limit = elem.find('limit')
                limit.attrib['lower'] = str(np.round(udh[3, _LLIM], 4))
                limit.attrib['upper'] = str(np.round(udh[3, _ULIM], 4))

            elif index == 'name' and elem.attrib[index] == '${prefix}joint_5':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[4, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[4, :], 8))
                limit = elem.find('limit')
                limit.attrib['lower'] = str(np.round(udh[4, _LLIM], 4))
                limit.attrib['upper'] = str(np.round(udh[4, _ULIM], 4))

            elif index == 'name' and elem.attrib[index] == '${prefix}joint_6':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[5, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[5, :], 8))
                limit = elem.find('limit')
                limit.attrib['lower'] = str(np.round(udh[5, _LLIM], 4))
                limit.attrib['upper'] = str(np.round(udh[5, _ULIM], 4))

            elif index == 'name' and elem.attrib[index] == '${prefix}flange_fixed_joint':
                origin = elem.find('origin')
                origin.attrib['xyz'] = str_from_nparray(np.round(xyzs[6, :], 8))
                origin.attrib['rpy'] = str_from_nparray(np.round(rpys[6, :], 8))
    pretty_xml(link_root, '  ', '\n')
    link_data = ET.tostring(link_root, encoding='UTF-8')
    #print '---'
    #print(link_data)
    #print '---'
    link_data = link_data.replace('ns0', 'xacro')
    link_data = link_data.replace(link_head, '', 1)
    link_data = link_data.replace(link_start, link_tag, 1)
    link_data = link_data.replace(link_end, link_tag, 1)
    #print(link_data)
    #print '---'
    robot_data = data[0] + link_data + data[2]
    return robot_data

def rludh_generator(file, xyzs, rpys, udh):
    #tree = ET.parse(file)
    f = open(file, 'r')
    data = f.read()
    root = ET.fromstring(data)
    i = 0
    #for elem in tree.iter(tag='fixed'):
    for elem in root.findall('joint'):
        for index in elem.attrib:
            if i < 7 and index == 'id' and elem.attrib[index] == 'fixed' + str(i):
                rotation = elem.find('rotation')
                rotation.find('x').text = str(np.round(math.degrees(rpys[i, 0]), 6))
                rotation.find('y').text = str(np.round(math.degrees(rpys[i, 1]), 6))
                rotation.find('z').text = str(np.round(math.degrees(rpys[i, 2]), 6))
                translation = elem.find('translation')
                translation.find('x').text = str(np.round(1000.0 * xyzs[i, 0], 6))
                translation.find('y').text = str(np.round(1000.0 * xyzs[i, 1], 6))
                translation.find('z').text = str(np.round(1000.0 * xyzs[i, 2], 6))
                i += 1
    pretty_xml(root, '  ', '\n')
    return ET.tostring(root, encoding='UTF-8')

def mdl_gen(file_out, file_tmpl, mdl_type, tm_dh, tm_delta_dh):

    if not tm_dh.startswith('DHTable={') or not tm_dh.endswith('}'):
        print('invalid tm_dh')
        return
    if not tm_delta_dh.startswith('DeltaDH={') or not tm_delta_dh.endswith('}'):
        print('invalid tm_delta_dh')
        return

    dh_strs = tm_dh[9:-1].split(',')
    ddh_strs = tm_delta_dh[9:-1].split(',')

    if len(dh_strs) != 42:
        print('invalid tm_dh')
        return
    if len(ddh_strs) != 30:
        print('invalid tm_delta_dh')
        return

    dh = [float(i) for i in dh_strs]
    ddh = [float(i) for i in ddh_strs]
    print(dh_strs)
    print(ddh_strs)
    data = ''

    if mdl_type == 'urdf':
        udh = urdf_DH_from_tm_DH(dh, ddh)
        xyzs, rpys = xyzrpys_from_urdf_DH(udh)
        data = urdf_generator(file_tmpl, xyzs, rpys, udh)

    elif mdl_type == 'rludh':
        udh = urdf_DH_from_tm_DH(dh, ddh)
        xyzs, rpys = xyzrpys_from_urdf_DH(udh)
        data = rludh_generator(file_tmpl, xyzs, rpys, udh)

    elif mdl_type == 'rlfdh':
        print('mdl_type rlfdh no implement')
        sys.exit(0)
    elif mdl_type == 'rlrdh':
        print('mdl_type rlrdh no implement')
        sys.exit(0)
    else:
        print('invalid mdl_type')
        sys.exit(0)

    #tree.write(file_out, encoding='UTF-8')
    #print(data)
    f = open(file_out, 'w')
    f.write(data)

def _mdl_gen():
    if (len(sys.argv) < 6):
        print('need 5 argument:')
        #sys.exit(0)
    tm_DH = 'DHTable={0.,-90.,0.,145.2,0.,-270.,270,-90.,0.,429.,0.,0.,-180.,180.,0.,0.,411.5,0.,0.,-158.,158.,90.,90.,0.,-122.3,0.,-180.,180.,0.,90.,0.,106.,0.,-180.,180.,0.,0.,0.,113.15,0.,-270.,270.}'
    tm_DeltaDH = 'DeltaDH={-0.001059821,0.02508766,0.009534874,0.,0.001116668,0.06614932,0.308224,0.0287381,0.06797475,-0.0319523,0.3752921,0.06614756,-0.006998898,0.06792655,-0.06083903,0.02092069,0.02965812,-0.1331249,0.06793034,0.02077797,0.08265772,0.03200645,0.01835932,0.06145732,0.08273286,0.6686108,0.6972408,-0.1793097,-0.0794057,1.425708}'
    #tm_DeltaDH = 'DeltaDH={0.,0.,0.,0.,0.,0.,0.,0.,0.,0.0,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.}'
    #mdl_gen('tm5_udh_output.urdf', 'tm5_udh_tmpl.urdf', 'urdf', tm_DH, tm_DeltaDH)
    mdl_gen('macro.tm5-900-mdh.urdf.xacro', '../../xacro/macro.tm5-900.urdf.xacro', 'urdf', tm_DH, tm_DeltaDH)
    #mdl_gen('tm5-udh-output.xml', 'tmr-tm5-udh-tmpl.xml', 'rludh', tm_DH, tm_DeltaDH)
    #mdl_gen(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])

if __name__ == '__main__':
    _mdl_gen()