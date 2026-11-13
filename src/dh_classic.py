import numpy as np
from .parameters import D1, D2, e2, aa, d4b, d5b, d6b

# Tabela 27 (Classic DH) - 6 DoF Curved Wrist
DH = [
    ( np.pi/2, 0,  D1   ),  # i=1
    ( np.pi,   D2, 0    ),  # i=2
    ( np.pi/2, 0, -e2   ),  # i=3
    ( 2*aa,    0, -d4b  ),  # i=4
    ( 2*aa,    0, -d5b  ),  # i=5
    ( np.pi,   0, -d6b  ),  # i=6
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

def physical_to_dh(q_physical):
    q1, q2, q3, q4, q5, q6 = q_physical
    return [
        -q1,              
        q2 - np.pi/2,     
        q3 + np.pi/2,     
        q4,               
        q5 - np.pi,       
        q6 + np.pi/2,     
    ]

def forward_kinematics_classic(q_physical):
    q_dh = physical_to_dh(q_physical)
    T = np.eye(4)
    for (alpha, a, d), theta in zip(DH, q_dh):
        T = T @ A(alpha, a, d, theta)
    return T

def forward_kinematics_full(q_physical):
    # Para o modelo Curved Wrist, a base do DH ja coincide com o link_base do ROS.
    return forward_kinematics_classic(q_physical)