import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.fk import forward_kinematics


# Configuração de teste
q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Cinemática direta
T = forward_kinematics(q)

print("q =", q)

print("\nT_0^6 =")
print(T)

print("\nPosition [x, y, z] =")
print(T[:3, 3])

print("\nRotation matrix R =")
print(T[:3, :3])