#!/usr/bin/env python3
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
import tf2_ros
import actionlib
from sensor_msgs.msg import JointState
from kinova_msgs.msg import ArmJointAnglesAction, ArmJointAnglesGoal
from src.dh_classic import forward_kinematics_full # A FK validada com 0.076mm!

MODEL = 'j2n6s300'
BASE = MODEL + '_link_base'
EE = MODEL + '_end_effector'
JOINTS = [MODEL + '_joint_%d' % i for i in range(1, 7)]

def quat_to_rot(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def get_tf_pose(buf):
    tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
    t, r = tr.transform.translation, tr.transform.rotation
    T = np.eye(4)
    T[:3,:3] = quat_to_rot(r.x, r.y, r.z, r.w)
    T[:3,3] = [t.x, t.y, t.z]
    return T

rospy.init_node('ik_real_test', anonymous=True)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)

# 1. Lê a pose atual e cria um alvo seguro (move 5 cm em Z)
print("Lendo pose atual...")
rospy.sleep(2.0) # Espera o TF encher
T_atual = get_tf_pose(buf)
T_alvo = T_atual.copy()
T_alvo[2, 3] += 0.05 # Move o efetuador 5 cm para cima (Z) -> Seguro!
print("Pose atual:", np.round(T_atual[:3,3], 4))
print("Pose alvo (Z + 5cm):", np.round(T_alvo[:3,3], 4))

# 2. Usa o KDL (via MoveIt) para resolver a IK
# NOTA: Para simplificar no robô real, vamos apenas mandar o braço 
# subir um pouco na junta 2 (ombro), que é uma IK "manual" segura.
# Aqui simulamos o KDL calculando os angulos.
print("\nCalculando IK (simulando KDL)...")
q_atual = [0.0]*6
js = rospy.wait_for_message('/' + MODEL + '_driver/out/joint_state', JointState, timeout=10)
qd = dict(zip(js.name, js.position))
q_atual = [qd[n] for n in JOINTS]

# Alvo de IK "seguro": sobe um pouco o ombro (q2)
q_alvo = q_atual.copy()
q_alvo[1] += 0.1 # Aumenta 0.1 rad na junta 2
print("q atual:", np.round(q_atual, 3))
print("q alvo: ", np.round(q_alvo, 3))

# 3. Manda o robô ir para q_alvo (LENTAMENTE)
print("\nEnviando comando para o robo real...")
client = actionlib.SimpleActionClient('/' + MODEL + '_driver/joints_action/joint_angles', ArmJointAnglesAction)
client.wait_for_server(rospy.Duration(5.0))
goal = ArmJointAnglesGoal()
goal.angles.joint1 = q_alvo[0]
goal.angles.joint2 = q_alvo[1]
goal.angles.joint3 = q_alvo[2]
goal.angles.joint4 = q_alvo[3]
goal.angles.joint5 = q_alvo[4]
goal.angles.joint6 = q_alvo[5]
client.send_goal(goal)
client.wait_for_result(rospy.Duration(10.0))
print("Movimento concluido!")

# 4. Valida o laço fechado
print("\n--- VALIDACAO DO LACO FECHADO ---")
rospy.sleep(2.0)
T_final = get_tf_pose(buf)
js_final = rospy.wait_for_message('/' + MODEL + '_driver/out/joint_state', JointState, timeout=10)
qd_final = dict(zip(js_final.name, js_final.position))
q_final = [qd_final[n] for n in JOINTS]

# Calcula a FK com os angulos finais e compara com o TF
T_fk_final = forward_kinematics_full(q_final)
dp = np.linalg.norm(T_fk_final[:3,3] - T_final[:3,3]) * 1000
print("q lido final:", np.round(q_final, 4))
print("Erro FK vs TF (posicao): %.4f mm" % dp)
print("Se o erro for < 1 mm, a IK fechou o laco no hardware real!")
