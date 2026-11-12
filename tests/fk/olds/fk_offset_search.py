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
from scipy.optimize import minimize
from src.dh_classic import forward_kinematics_classic

MODEL = 'j2s6s300'
BASE  = MODEL + '_link_base'
EE    = MODEL + '_end_effector'
JOINTS = [MODEL + '_joint_%d' % i for i in range(1, 7)]
CMD   = '/%s/effort_joint_trajectory_controller/command' % MODEL
CONFIGS = [
    [-0.8, 2.65, 1.05, -1.85, 1.25, -1.0],
    [ 1.2, 3.15, 1.55, -2.35, 1.55,  1.5],
    [-1.5, 2.9,  1.3,  -2.09, 1.4,   2.5],
    [ 2.0, 2.75, 1.1,  -1.9,  1.3,  -2.0],
]
VALID = [0.5, 3.0, 1.4, -2.1, 1.3, 0.3]   # pose de validação (não usada na otimização)
MOVE_TIME, SETTLE = 8.0, 5.0

def quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def tf_pose(buf):
    tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
    t, r = tr.transform.translation, tr.transform.rotation
    T = np.eye(4); T[:3,:3] = quat_to_rot(r.x, r.y, r.z, r.w); T[:3,3] = [t.x, t.y, t.z]
    return T

def move(pub, target):
    msg = JointTrajectory(); msg.joint_names = JOINTS
    pt = JointTrajectoryPoint(); pt.positions = list(target); pt.time_from_start = rospy.Duration(MOVE_TIME)
    msg.points = [pt]; pub.publish(msg); rospy.sleep(MOVE_TIME + SETTLE)

def inv_rigid(T):
    R = T[:3,:3].T; p = -R @ T[:3,3]
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = p
    return M

def collect(pub, buf, cfgs):
    data = []
    for cfg in cfgs:
        move(pub, cfg)
        js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
        qd = dict(zip(js.name, js.position))
        q = [qd[n] for n in JOINTS]
        data.append((q, tf_pose(buf)))
    return data

def objective(delta, data):
    d1, d4, d5 = delta
    corrs = []
    for q, T_tf in data:
        theta = [q[0]+d1, q[1], q[2], q[3]+d4, q[4]+d5, q[5]]
        T_dh = forward_kinematics_classic(theta)
        corrs.append(T_tf @ inv_rigid(T_dh))
    corrs = np.array(corrs)
    return float(np.var(corrs, axis=0).sum())

def report(delta, data, label):
    d1, d4, d5 = delta
    print('\n%s: delta = (%.4f, %.4f, %.4f)' % (label, d1, d4, d5))
    for i, (q, T_tf) in enumerate(data):
        theta = [q[0]+d1, q[1], q[2], q[3]+d4, q[4]+d5, q[5]]
        T_dh = forward_kinematics_classic(theta)
        T_corr = T_tf @ inv_rigid(T_dh)
        dp = np.linalg.norm(T_dh[:3,3] - T_tf[:3,3])*1000
        c = (np.trace(T_dh[:3,:3].T @ T_tf[:3,:3]) - 1)/2
        da = np.degrees(np.arccos(np.clip(c, -1, 1)))
        print('  pose %d: err_pos=%8.3f mm  err_ori=%8.3f °  T_corr.trans=%s' %
              (i+1, dp, da, np.round(T_corr[:3,3]*1000, 2)))

rospy.init_node('fk_offset_search', anonymous=True)
pub = rospy.Publisher(CMD, JointTrajectory, queue_size=1)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
while pub.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)
rospy.sleep(1.0)

print('Coletando 4 poses para otimizacao...')
data_fit = collect(pub, buf, CONFIGS)
print('Coletando pose de validacao...')
data_val = collect(pub, buf, [VALID])

print('\nOtimizando (delta1, delta4, delta5)...')
result = minimize(lambda d: objective(d, data_fit), x0=[0, 0, 0],
                  method='Nelder-Mead',
                  options={'xatol': 1e-5, 'fatol': 1e-10, 'maxiter': 2000})
print('Resultado:', result.x)
print('Objetivo final:', result.fun)

report([0, 0, 0], data_fit, 'SEM offset (baseline)')
report(result.x, data_fit, 'COM offset otimo (poses de fit)')
report(result.x, data_val, 'COM offset otimo (pose de VALIDACAO, fora do fit)')

print('\nVoltando para HOME...')
move(pub, [0.0, 2.9, 1.3, -2.09, 1.4, 0.0])