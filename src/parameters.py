# Parametros físicos do Jaco
import numpy as np

D1 = 0.2755
D2 = 0.41
D3 = 0.2073
D4 = 0.0741
D5 = 0.0741
D6 = 0.16
e2 = 0.0098  # Corrigido conforme manual Kinova Gen2 (6DoF Spherical), Tab. 23
WRIST_LENGTH = D3 + D4

aa = (30.0 * np.pi) / 180.0  # 30 graus em radianos
sa = np.sin(aa)
s2a = np.sin(2 * aa)

d4b = D3 + (sa / s2a) * D4
d5b = (sa / s2a) * D4 + (sa / s2a) * D5
d6b = (sa / s2a) * D5 + D6
