#!/usr/bin/env python3
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import itertools
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from src.dh_classic import forward_kinematics_classic

MODEL = 'j2s6s300'
BASE  = MODEL + '_link_base'
EE    = MODEL + '_end_effector'
JOINTS = [MODEL + '_joint_%d' % i for i in range(1, 7)]
CMD   = '/%s/effort_joint_trajectory_controller/command' % MODEL

def quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def tf_pose(buf):
    tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
    t, r = tr.transform.translation, tr.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_to_rot(r.x, r.y, r.z, r.w); T[:3, 3] = [t.x, t.y, t.z]
    return T

def move(pub, target):
    msg = JointTrajectory(); msg.joint_names = JOINTS
    pt = JointTrajectoryPoint(); pt.positions = list(target); pt.time_from_start = rospy.Duration(8.0)
    msg.points = [pt]; pub.publish(msg); rospy.sleep(14.0)

def erro(T_dh, T_tf):
    dp = np.linalg.norm(T_dh[:3, 3] - T_tf[:3, 3]) * 1000
    c  = (np.trace(T_dh[:3, :3].T @ T_tf[:3, :3]) - 1) / 2
    da = np.degrees(np.arccos(np.clip(c, -1, 1)))
    return dp, da

def test_signs(q, T_tf, label):
    print('\n--- %s: q = %s' % (label, [round(x, 3) for x in q]))
    print('%15s | %10s | %10s' % ('(s1,s4,s5)', 'erro_pos', 'erro_ori'))
    print('-' * 45)
    best = (None, 1e18)
    for s1, s4, s5 in itertools.product([1, -1], repeat=3):
        q_mod = [s1*q[0], q[1], q[2], s4*q[3], s5*q[4], q[5]]
        T_dh = forward_kinematics_classic(q_mod)
        dp, da = erro(T_dh, T_tf)
        total = dp**2 + da**2
        marker = ''
        if total < best[1]:
            best = ((s1, s4, s5), total); marker = ' *'
        print('%3d %3d %3d     | %8.2f mm | %8.2f °%s' % (s1, s4, s5, dp, da, marker))
    print('Melhor: %s' % str(best[0]))

rospy.init_node('fk_sign_search', anonymous=True)
pub = rospy.Publisher(CMD, JointTrajectory, queue_size=1)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
while pub.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)
rospy.sleep(1.0)

# Pose 1: HOME
move(pub, [0.0, 2.9, 1.3, -2.09, 1.4, 0.0])
js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
qd = dict(zip(js.name, js.position))
q_home = [qd[n] for n in JOINTS]
test_signs(q_home, tf_pose(buf), 'HOME')

# Pose 2: PERT (q1, q4, q5 != 0)
move(pub, [0.5, 2.9, 1.3, -1.59, 1.9, 0.5])
js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
qd = dict(zip(js.name, js.position))
q_pert = [qd[n] for n in JOINTS]
test_signs(q_pert, tf_pose(buf), 'PERT')

# Volta a HOME
move(pub, [0.0, 2.9, 1.3, -2.09, 1.4, 0.0])