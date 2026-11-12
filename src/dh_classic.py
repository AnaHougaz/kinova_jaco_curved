import numpy as np
from .parameters import D1, D2, D3, D4, D5, D6, e2, aa, d4b, d5b, d6b

#tabela DH (falta o angulo na 4 coluna, mas a 4 coluna entra no loop for (alpha, a, d), theta in zip(DH, q_dh))
DH = [
    ( np.pi/2, 0,  D1   ),   # i=1
    ( np.pi,   D2, 0    ),   # i=2
    ( np.pi/2, 0, -e2   ),   # i=3
    ( 2*aa,    0, -d4b  ),   # i=4 (alpha = 2*aa)
    ( 2*aa,    0, -d5b  ),   # i=5 (alpha = 2*aa)
    ( np.pi,   0, -d6b  ),   # i=6
]

def A(alpha, a, d, theta):
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s*ca,  s*sa, a*c],
        [s,  c*ca, -c*sa, a*s],
        [0,  sa,    ca,   d  ],
        [0,  0,     0,    1  ],
    ])

# Table 30: Transformation from DH algorithm to robot physical angles
# page 54 of notes/jaco_manual.pdf
# Offsets de junta
def physical_to_dh(q_physical):
    """Converte angulos fisicos do ROS (URDF) para angulos do algoritmo DH.
       Baseado na Tabela 28 do manual Kinova Gen2 (pag. 53) - Curved Wrist."""
    q1, q2, q3, q4, q5, q6 = q_physical
    return [
        -q1,              # Q1_fisico = -Q1_dh  =>  Q1_dh = -Q1_fisico
        q2 - np.pi/2,     # Q2_fisico = Q2_dh + 90  =>  Q2_dh = Q2_fisico - 90
        q3 + np.pi/2,     # Q3_fisico = Q3_dh - 90  =>  Q3_dh = Q3_fisico + 90
        q4,               # Q4_fisico = Q4_dh
        q5 - np.pi,       # Q5_fisico = Q5_dh + 180 =>  Q5_dh = Q5_fisico - 180
        q6 + np.pi/2,     # Q6_fisico = Q6_dh - 90  =>  Q6_dh = Q6_fisico + 90
    ]

def forward_kinematics_classic(q_physical):
    """FK a partir dos angulos fisicos. Aplica a conversao para DH antes da matriz."""
    q_dh = physical_to_dh(q_physical)
    T = np.eye(4)   #cria matriz 4x4 vaziaS
    for (alpha, a, d), theta in zip(DH, q_dh):  #Pegando a linha 1 da lista DH (que tem as 3 colunas fixas) e o primeiro ângulo de q_dh (que é a 4ª coluna).
        T = T @ A(alpha, a, d, theta)
    return T
    

# Matriz de transformação homogênea (4x4):
#
#       | x_x  y_x  z_x  P_x |
# T  =  | x_y  y_y  z_y  P_y |
#       | x_z  y_z  z_z  P_z |
#       |  0    0    0    1  |

def T_base_to_0():
    """Transformacao fixa da base URDF para o frame 0 do DH.
       Baseado na Pagina 56 do manual Kinova Gen2.
       Rotacao de 180 graus em X e translacao de 551 mm em Z."""
    # A pose do DH em relação ao do ROS tem exatamente Rx(180)e Tz(0.551)
    M = np.eye(4)
    # Rotacao de 180 em torno X: [[1,0,0],[0,-1,0],[0,0,-1]]
    # M[:3, :3] é matriz de rotação 
    # segue a matriz genérica de rotação do eixo x
    # O manual diz que o eixo Z do frame 0 da matemática 
    # aponta para baixo, enquanto o Z do ROS aponta para cima.
    M[:3, :3] = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
    ])
    # M[:3, 3] vetor de translação  
    M[:3, 3] = [0.0, 0.0, 0.551024]  
    # Manual diz: a origem do frame 0 está 551 milímetros (0,551 m) 
    # ACIMA da base física do robô (que é o link_base do ROS).
    return M

def forward_kinematics_full(q_physical):
    """FK completa: mapeia angulos fisicos, aplica DH classica e corrige a base."""
#    T_dh = forward_kinematics_classic(q_physical)
    return forward_kinematics_classic(q_physical)
