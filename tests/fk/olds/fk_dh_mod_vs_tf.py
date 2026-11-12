#!/usr/bin/env python3
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from src.fk import forward_kinematics                     # Tabela 6, modificada, termina em link_6
from src.dh_classic import forward_kinematics_classic     # Tabela 5, clássica, termina no EE

MODEL = 'j2s6s300'
BASE  = MODEL + '_link_base'
EE    = MODEL + '_end_effector'

def quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])

def rot_rpy(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

# Transformacao fixa link_6 -> end_effector, do URDF:
#   <origin xyz="0 0 -0.16" rpy="pi 0 pi/2">
T_6_EE = np.eye(4)
T_6_EE[:3, :3] = rot_rpy(np.pi, 0, np.pi/2)
T_6_EE[:3, 3]  = [0.0, 0.0, -0.16]

rospy.init_node('fk_dh_mod_vs_tf', anonymous=True)

js = rospy.wait_for_message('/' + MODEL + '/joint_states', JointState, timeout=10)
q_map = dict(zip(js.name, js.position))
q = [q_map['%s_joint_%d' % (MODEL, i)] for i in range(1, 7)]
print('q (rad):', [round(x, 5) for x in q])

T_classic = forward_kinematics_classic(q)          # Tab 5, ja termina no EE
T_mod_l6  = forward_kinematics(q)                  # Tab 6, termina em link_6
T_mod_ee  = T_mod_l6 @ T_6_EE                      # Tab 6 + offset ate EE

buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
t, r = tr.transform.translation, tr.transform.rotation
T_tf = np.eye(4)
T_tf[:3, :3] = quat_to_rot(r.x, r.y, r.z, r.w)
T_tf[:3, 3]  = [t.x, t.y, t.z]

def report(nome, T):
    dp = np.linalg.norm(T[:3, 3] - T_tf[:3, 3]) * 1000
    c  = (np.trace(T[:3, :3].T @ T_tf[:3, :3]) - 1) / 2
    da = np.degrees(np.arccos(np.clip(c, -1, 1)))
    print('%-28s pos=%s  err_pos=%8.3f mm  err_ori=%8.3f graus'
          % (nome, np.round(T[:3, 3], 5), dp, da))

print('TF posicao:'.ljust(28), np.round(T_tf[:3, 3], 5))
report('DH classica (Tab 5)',       T_classic)
report('DH modificada (Tab 6, l6)', T_mod_l6)
report('DH modificada + T6->EE',    T_mod_ee)