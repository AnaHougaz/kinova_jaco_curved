'''
Calcula a FK usando o URDF e depois compara com o TF ROS 
'''
#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

MODEL = 'j2n6s300'
BASE = MODEL + '_link_base'
EE = MODEL + '_end_effector'

# cria matriz de rotação baseado nos angulos roll, pitch e yaw
def rot_rpy(r, p, y):
    # convenção do URDF: R = Rz(y) @ Ry(p) @ Rx(r)
    # aqui pra transformar o urdf em matriz homogenea ele pega as matrizes de 
    # rotação em x,y e z e multiplica em ordem "inversa" assim: rz @ ry @rx
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# Recebe o eixo ([0,0,1], por exemplo) e o ângulo q, e usa a fórmula de Rodrigues para gerar a matriz de rotação 3x3 móvel.
def rot_axis(axis, q):
    # rotação de q radianos em torno de um eixo (fórmula de Rodrigues)
    a = np.array(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)

# Aqui ela pega a matriz de rotação 3x3 (R) e o vetor de translação (p), e devolve a matriz homogênea T 4x4
def transform(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M

def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])

# inicia um novo nó fk_urdf_vs_tf
#o anonymuos coloca uns numeros aleatorios no final do nome do nó pra se abrir outro terminal, o novo não impede o anterior, isso porque o ros não permite dois nós com o mesmo nome
rospy.init_node('fk_urdf_vs_tf', anonymous=True)
robot = URDF.from_parameter_server('/robot_description')

# monta a cadeia de juntas do efetuador até a base e inverte
by_child = {j.child: j for j in robot.joints}
chain = [] #lista de juntas vazia
link = EE #end-effector
while link != BASE:
    j = by_child[link]
    chain.append(j)
    link = j.parent
chain.reverse() #o while parte da garra até a base, então aqui voltamos ao sentido normal
print('Cadeia:', [j.name for j in chain])

#pede que ros envie uma mensagem do topico do robo 
#apenas pra verificar se ele está certinho, caso não ele interrompe o script depois de 10 seg esperando
js = rospy.wait_for_message('/' + MODEL + '_driver/out/joint_state', JointState, timeout=10)
q = dict(zip(js.name, js.position)) #junta as juntas e suas posições
print('q (rad):', [round(q[MODEL + '_joint_%d' % i], 5) for i in range(1, 7)])

T_fk = np.eye(4) # Cria a matriz 4x4
for j in chain:  # Percorre as juntas do chão até a garra.
    o = j.origin # Pega o "origin" da junta atual no URDF.
    
    # PASSO 1: Junta fixa.
    # rot_rpy(*o.rpy) cria a matriz de rotação 3x3 a partir dos ângulos RPY.
    # transform(R, p) pega essa rotação e a posição (o.xyz) e monta a matriz T 4x4 fixa.
    # T_fk = T_fk @ (matriz fixa)
    T_fk = T_fk @ transform(rot_rpy(*o.rpy), o.xyz)
    
    # PASSO 2: AJunta móvel
    if j.type in ('revolute', 'continuous'):
        # rot_axis(j.axis, q[j.name]) cria a matriz de rotação 3x3 em torno do eixo da junta.
        # O transform() junta essa rotação com a translação [0,0,0] (porque a junta gira em torno do próprio centro).
        # T_fk = T_fk @ (matriz de rotação da junta)
        T_fk = T_fk @ transform(rot_axis(j.axis, q[j.name]), [0, 0, 0])

buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf)
tr = buf.lookup_transform(BASE, EE, rospy.Time(0), rospy.Duration(5.0))
t, r = tr.transform.translation, tr.transform.rotation
T_tf = transform(quat_to_rot(r.x, r.y, r.z, r.w), [t.x, t.y, t.z])

dp = np.linalg.norm(T_fk[:3, 3] - T_tf[:3, 3])
c = (np.trace(T_fk[:3, :3].T @ T_tf[:3, :3]) - 1) / 2
da = np.degrees(np.arccos(np.clip(c, -1, 1)))
print('Posição FK (m):', np.round(T_fk[:3, 3], 5))
print('Posição TF (m):', np.round(T_tf[:3, 3], 5))
print('Erro de posição (mm): %.4f' % (dp * 1000))
print('Erro de orientação (graus): %.4f' % da)
