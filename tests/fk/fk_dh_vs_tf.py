'''
A minha implementação da cinemática direta usando parâmetros DH está calculando a mesma posição e orientação que o ROS/TF calcula para o robô?
'''

import os, sys
# Como o script esta em my_workspace/tests/fk/, precisamos subir 3 niveis
# para chegar em my_workspace/
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from src.dh_classic import forward_kinematics_full


MODEL = 'j2n6s300'
BASE = MODEL + '_link_base'
EE   = MODEL + '_end_effector'

def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])

rospy.init_node('fk_dh_full_vs_tf', anonymous=True)
js = rospy.wait_for_message('/' + MODEL + '/joint_states', JointState, timeout=10)
q_map = dict(zip(js.name, js.position))
q = [q_map['%s_joint_%d' % (MODEL, i)] for i in range(1, 7)]
print('q (rad):', [round(x, 5) for x in q])

T_dh = forward_kinematics_full(q)

buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf)
tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
t, r = tr.transform.translation, tr.transform.rotation
T_tf = np.eye(4)
T_tf[:3, :3] = quat_to_rot(r.x, r.y, r.z, r.w)
T_tf[:3, 3]  = [t.x, t.y, t.z]

dp = np.linalg.norm(T_dh[:3, 3] - T_tf[:3, 3]) * 1000
c  = (np.trace(T_dh[:3, :3].T @ T_tf[:3, :3]) - 1) / 2
da = np.degrees(np.arccos(np.clip(c, -1, 1)))

print('DH full posicao (m):', np.round(T_dh[:3, 3], 5))
print('TF posicao (m):     ', np.round(T_tf[:3, 3], 5))
print('Erro de posicao (mm):      %.4f' % dp)
print('Erro de orientacao (graus): %.4f' % da)