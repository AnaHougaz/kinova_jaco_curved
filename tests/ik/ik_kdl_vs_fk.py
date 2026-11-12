'''
Pega um alvo, manda o KDL do ROS resolver a IK, e usa a FK para ver se o robô chega no lugar certo
'''
#!/usr/bin/env python3
import os, sys
# O arquivo esta em my_workspace/tests/ik/ik_kdl_vs_fk.py
# Precisamos subir 3 niveis para chegar em my_workspace/
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np
import rospy
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from src.dh_classic import forward_kinematics_full # Nossa FK validada!

MODEL = 'j2n6s300'
BASE = MODEL + '_link_base'
EE   = MODEL + '_end_effector'

# 1. Carrega o URDF e constroi a cadeia KDL
print('Carregando URDF...')
robot = URDF.from_parameter_server('/robot_description')
success, kdl_tree = treeFromUrdfModel(robot)
if not success:
    print('Erro ao criar a arvore KDL'); sys.exit(1)

# Extrai a cadeia da base ate o efetuador
kdl_chain = kdl_tree.getChain(BASE, EE)
print('Cadeia KDL extraida: %d juntas' % kdl_chain.getNrOfJoints())

# 2. Configura o solver de IK (Levenberg-Marquardt)
# Ele e robusto para robos com punho esferico
ik_solver = PyKDL.ChainIkSolverPos_LMA(kdl_chain, 1e-6, 500, 1e-10)

# 3. Gera uma pose alvo usando a NOSSA FK (Gabarito)
q_target = np.array([0.5, 2.9, 1.3, -2.09, 1.4, 0.5]) # Angulos aleatorios dentro do limite
T_target_np = forward_kinematics_full(q_target)

# Converte a matriz numpy para o Frame do KDL
T_target_kdl = PyKDL.Frame()
for i in range(3):
    for j in range(3):
        T_target_kdl.M[i, j] = T_target_np[i, j]
    T_target_kdl.p[i] = T_target_np[i, 3]

# 4. Pede para o KDL resolver a IK
q_init = PyKDL.JntArray(kdl_chain.getNrOfJoints()) # Chute inicial (zeros)
q_kdl = PyKDL.JntArray(kdl_chain.getNrOfJoints())

print('Rodando IK do KDL...')
ret = ik_solver.CartToJnt(q_init, T_target_kdl, q_kdl)

if ret < 0:
    print('KDL falhou em encontrar solucao (codigo %d)' % ret)
    sys.exit(1)

# Extrai o resultado do KDL para numpy
q_kdl_np = np.array([q_kdl[i] for i in range(kdl_chain.getNrOfJoints())])
print('q_alvo (nossa FK):', np.round(q_target, 4))
print('q_kdl  (KDL):     ', np.round(q_kdl_np, 4))

# 5. VALIDACAO: Joga o q do KDL na nossa FK e compara com a pose alvo
T_check_np = forward_kinematics_full(q_kdl_np)

dp = np.linalg.norm(T_check_np[:3, 3] - T_target_np[:3, 3]) * 1000
c  = (np.trace(T_check_np[:3, :3].T @ T_target_np[:3, :3]) - 1) / 2
da = np.degrees(np.arccos(np.clip(c, -1, 1)))

print('\n--- RESULTADO DO LACO FECHADO ---')
print('Pose Alvo   (m):', np.round(T_target_np[:3, 3], 5))
print('Pose KDL    (m):', np.round(T_check_np[:3, 3], 5))
print('Erro de posicao (mm):     %.6f' % dp)
print('Erro de orientacao (graus): %.6f' % da)

if dp < 0.1 and da < 0.1:
    print('\nSUCESSO: O KDL encontrou uma solucao valida e a nossa FK confirmou!')
else:
    print('\nATENCAO: Erro alto. Verifique os limites ou o chute inicial do KDL.')