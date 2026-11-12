#!/usr/bin/env python3
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from src.dh_classic import forward_kinematics_classic

MODEL = 'j2n6s300'
BASE = MODEL + '_link_base'
EE = MODEL + '_end_effector'
JOINTS = [MODEL + '_joint_%d' % i for i in range(1, 7)]

def quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def inv_rigid(T):
    R = T[:3,:3].T; p = -R @ T[:3,3]
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = p
    return M

rospy.init_node('print_tcorr_real', anonymous=True)
js = rospy.wait_for_message('/' + MODEL + '_driver/out/joint_state', JointState, timeout=10)
qd = dict(zip(js.name, js.position))
q = [qd[n] for n in JOINTS]
print('q (rad):', [round(x, 4) for x in q])

T_dh = forward_kinematics_classic(q)

buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
t, r = tr.transform.translation, tr.transform.rotation
T_tf = np.eye(4)
T_tf[:3,:3] = quat_to_rot(r.x, r.y, r.z, r.w)
T_tf[:3,3] = [t.x, t.y, t.z]

T_corr = T_tf @ inv_rigid(T_dh)
np.set_printoptions(suppress=True, precision=4)
print('\nT_corr = T_tf @ inv(T_dh):')
print(T_corr)

R = T_corr[:3,:3]
print('\nR_corr (3x3):')
print(R)
print('\ntranslacao T_corr (mm):', np.round(T_corr[:3,3]*1000, 2))
