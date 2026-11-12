#!/usr/bin/env python3
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
from src.dh_classic import forward_kinematics_classic

MODEL = 'j2s6s300'
BASE  = MODEL + '_link_base'
EE    = MODEL + '_end_effector'
JOINTS = [MODEL + '_joint_%d' % i for i in range(1, 7)]
CMD   = '/%s/effort_joint_trajectory_controller/command' % MODEL
HOME  = [0.0, 2.9, 1.3, -2.09, 1.4, 0.0]
CONFIGS = [
    [-0.8, 2.65, 1.05, -1.85, 1.25, -1.0],
    [ 1.2, 3.15, 1.55, -2.35, 1.55,  1.5],
    [-1.5, 2.9,  1.3,  -2.09, 1.4,   2.5],
    [ 2.0, 2.75, 1.1,  -1.9,  1.3,  -2.0],
]
MOVE_TIME = 8.0
SETTLE    = 6.0

def quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])

def tf_pose(buf):
    tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
    t, r = tr.transform.translation, tr.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(r.x, r.y, r.z, r.w)
    T[:3, 3]  = [t.x, t.y, t.z]
    return T

def invert_rigid(T):
    # inversa de transformacao rígida: R^T e -R^T p (mais estável que np.linalg.inv)
    R_inv = T[:3, :3].T
    p_inv = -R_inv @ T[:3, 3]
    M = np.eye(4); M[:3, :3] = R_inv; M[:3, 3] = p_inv
    return M

def rot_angle_axis(R):
    # devolve (theta em rad, eixo unitario)
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    theta = np.arccos(c)
    if theta < 1e-8:
        return 0.0, np.array([0.0, 0.0, 1.0])
    if np.pi - theta < 1e-4:
        # caso especial theta ~ pi: eixo = coluna de R+I
        M = R + np.eye(3)
        k = int(np.argmax(np.linalg.norm(M, axis=0)))
        axis = M[:, k].copy()
        j = int(np.argmax(np.abs(axis)))
        if axis[j] < 0:
            axis = -axis
        return theta, axis / np.linalg.norm(axis)
    axis = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*np.sin(theta))
    return theta, axis

def move(pub, target):
    msg = JointTrajectory()
    msg.joint_names = JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = list(target)
    pt.time_from_start = rospy.Duration(MOVE_TIME)
    msg.points = [pt]
    pub.publish(msg)
    rospy.sleep(MOVE_TIME + SETTLE)

# --- checagem de limites (mesma logica do fk_multi_config) ---
robot = URDF.from_parameter_server('/robot_description')
lims = {j.name: (j.limit.lower, j.limit.upper) for j in robot.joints
        if j.type == 'revolute' and j.limit is not None}
for cfg in CONFIGS + [HOME]:
    for n, v in zip(JOINTS, cfg):
        if n in lims and not (lims[n][0] <= v <= lims[n][1]):
            print('ABORTADO: %s=%.3f fora do limite %s' % (n, v, lims[n]))
            sys.exit(1)

rospy.init_node('fk_corr_diag', anonymous=True)
pub = rospy.Publisher(CMD, JointTrajectory, queue_size=1)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
while pub.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)
rospy.sleep(1.0)

print('Diagnostico: T_corr = T_tf @ inv(T_dh)  (por pose)')
print('Objetivo: ver se T_corr e CONSTANTE nas 4 poses.\n')

rows = []
for k, cfg in enumerate(CONFIGS, 1):
    print('--- Config %d: %s' % (k, cfg))
    move(pub, cfg)
    js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
    qd = dict(zip(js.name, js.position))
    q  = [qd[n] for n in JOINTS]

    T_dh  = forward_kinematics_classic(q)
    T_tf  = tf_pose(buf)
    T_cor = T_tf @ invert_rigid(T_dh)
    t_cor = T_cor[:3, 3]
    theta, axis = rot_angle_axis(T_cor[:3, :3])

    print('  T_dh  pos =', np.round(T_dh[:3, 3], 5))
    print('  T_tf  pos =', np.round(T_tf[:3, 3], 5))
    print('  T_corr.trans (mm)   =', np.round(t_cor * 1000, 3))
    print('  T_corr.angulo (grau)= %.3f' % np.degrees(theta))
    print('  T_corr.eixo         =', np.round(axis, 4))
    print()
    rows.append({'t': t_cor*1000, 'theta': np.degrees(theta), 'axis': axis})

t_arr   = np.array([r['t']     for r in rows])
th_arr  = np.array([r['theta'] for r in rows])
ax_arr  = np.array([r['axis']  for r in rows])

print('=== COMPARACAO ENTRE POSES ===')
print('trans (mm):')
for k, t in enumerate(t_arr, 1):
    print('  pose %d: %s' % (k, np.round(t, 3)))
print('  media :', np.round(t_arr.mean(axis=0), 3))
print('  desvio:', np.round(t_arr.std(axis=0), 3))
print()
print('angulo (graus):')
for k, a in enumerate(th_arr, 1):
    print('  pose %d: %.3f' % (k, a))
print('  media : %.3f  desvio: %.3f' % (th_arr.mean(), th_arr.std()))
print()
print('eixo:')
for k, ax in enumerate(ax_arr, 1):
    print('  pose %d: %s' % (k, np.round(ax, 4)))
print('  media :', np.round(ax_arr.mean(axis=0), 4))
print('  desvio:', np.round(ax_arr.std(axis=0), 4))
print()

if t_arr.std(axis=0).max() < 5.0 and th_arr.std() < 2.0:
    print('VEREDITO: T_corr e CONSTANTE => DH correta, so offset de frame.')
    print('  Frame fixo de correcao (media):')
    print('    translacao (mm):', np.round(t_arr.mean(axis=0), 3))
    print('    angulo (graus):  %.3f' % th_arr.mean())
    print('    eixo:            ', np.round(ax_arr.mean(axis=0), 4))
    print('  => evidencia L2 fechada. Proximo: incorporar T_corr na FK e re-testar.')
else:
    print('VEREDITO: T_corr VARIA com a pose => parametro DH errado ou convencao por junta.')
    print('  Proximo: decompor junta por junta (rodar so q1, depois q1+q2, etc.)')

print('\nVoltando a HOME...')
move(pub, HOME)