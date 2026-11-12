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

HOME = [0.0, 2.9, 1.3, -2.09, 1.4, 0.0]
# Perturbacoes a testar: (junta indexada em 0, delta)
PROBES = [
    (0, +0.5), (0, -0.5),    # q1
    (1, +0.5), (1, -0.5),    # q2
    (2, +0.5), (2, -0.5),    # q3
    (3, +0.5), (3, -0.5),    # q4
    (4, +0.5), (4, -0.5),    # q5
    (5, +0.5), (5, -0.5),    # q6
]
MOVE_TIME = 8.0
SETTLE    = 5.0

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

def move(pub, target):
    msg = JointTrajectory()
    msg.joint_names = JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = list(target)
    pt.time_from_start = rospy.Duration(MOVE_TIME)
    msg.points = [pt]
    pub.publish(msg)
    rospy.sleep(MOVE_TIME + SETTLE)

def erro(T_dh, T_tf):
    dp = np.linalg.norm(T_dh[:3, 3] - T_tf[:3, 3]) * 1000
    c  = (np.trace(T_dh[:3, :3].T @ T_tf[:3, :3]) - 1) / 2
    da = np.degrees(np.arccos(np.clip(c, -1, 1)))
    return dp, da

robot = URDF.from_parameter_server('/robot_description')
lims = {j.name: (j.limit.lower, j.limit.upper) for j in robot.joints
        if j.type == 'revolute' and j.limit is not None}

rospy.init_node('fk_joint_isolation', anonymous=True)
pub = rospy.Publisher(CMD, JointTrajectory, queue_size=1)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
while pub.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)
rospy.sleep(1.0)

print('Base: HOME =', HOME)
print('%8s | %6s | %10s | %10s' % ('junta', 'delta', 'erro_pos', 'erro_ori'))
print('-' * 45)
results = []

# Baseline HOME
move(pub, HOME)
js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
qd = dict(zip(js.name, js.position))
q  = [qd[n] for n in JOINTS]
T_dh = forward_kinematics_classic(q); T_tf = tf_pose(buf)
dp0, da0 = erro(T_dh, T_tf)
print('%8s | %6s | %8.2f mm | %6.2f °' % ('HOME', '0.0', dp0, da0))
print('-' * 45)

for i, delta in PROBES:
    target = list(HOME)
    target[i] += delta
    # checagem de limite
    ok = True
    for n, v in zip(JOINTS, target):
        if n in lims and not (lims[n][0] <= v <= lims[n][1]):
            print('  q%d=%.2f fora de limite, pulando.' % (i+1, target[i]))
            ok = False; break
    if not ok:
        continue
    move(pub, target)
    js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
    qd = dict(zip(js.name, js.position))
    q  = [qd[n] for n in JOINTS]
    T_dh = forward_kinematics_classic(q); T_tf = tf_pose(buf)
    dp, da = erro(T_dh, T_tf)
    print('%8s | %+6.2f | %8.2f mm | %6.2f °' % ('q%d' % (i+1), delta, dp, da))
    results.append((i+1, delta, dp, da))
    # volta para HOME entre probes (evita acumulo)
    move(pub, HOME)

print('\nInterpretacao:')
print('  Se q1(+0.5) e q1(-0.5) dao erros de MAGNITUDE parecida -> offset em q1.')
print('  Se dao erros OPOSTOS (um muito maior que o outro) -> sinal invertido em q1.')
print('  Se q2..q5 derem erro ~HOME -> estrutura do meio OK (parametros a/d/alpha certos).')
print('  Junta cujo |delta| provoca maior erro eh a suspeita principal.')

# Conclusão: q2, q3, q6 estão com a convenção de junta correta entre o artigo e o URDF. q1, q4, q5 têm a convenção errada. Isso é ouro para o diagnóstico.

# Base: `HOME = [0.0, 2.9, 1.3, -2.09, 1.4, 0.0]`

# | Junta | Delta | Erro de posição | Erro de orientação |
# |:-----:|:-----:|----------------:|--------------------:|
# | HOME  |  0.0  | 37.84 mm        | 105.70°             |
# | q1    | +0.50 | 233.32 mm       | 124.22°             |
# | q1    | -0.50 | 270.72 mm       | 107.32°             |
# | q2    | +0.50 | 39.30 mm        | 105.64°             |
# | q2    | -0.50 | 36.26 mm        | 105.75°             |
# | q3    | +0.50 | 322.99 mm       | 105.75°             |
# | q3    | -0.50 | 255.72 mm       | 105.63°             |
# | q4    | +0.50 | 274.23 mm       | 91.68°              |
# | q4    | -0.50 | 161.22 mm       | 137.30°             |
# | q5    | +0.50 | 289.14 mm       | 109.08°             |
# | q5    | -0.50 | 223.78 mm       | 122.46°             |
# | q6    | +0.50 | 37.88 mm        | 105.70°             |
# | q6    | -0.50 | 37.89 mm        | 105.69°             |

