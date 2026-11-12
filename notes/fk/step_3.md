# Calcular a FK a partir do URDF e comparar com o TF

**Objetivo:** montar a FK usando **os números do modelo** (e não os do artigo) e compará-la com o TF no mesmo conjunto de ângulos. É o seu primeiro teste de FK, com posição e orientação, e responde aos itens 5 a 7 da sua seção de FK.

**Lógica do script:** a FK é o produto, em ordem da base ao efetuador, de uma transformação fixa (`origin`) e, se a junta for rotativa, de uma rotação de `q` radianos em torno do `axis`. O script lê os ângulos do `joint_states`, calcula a pose e a compara com a do `tf2`, medindo o erro de posição (distância em mm) e de orientação (ângulo entre as duas rotações, em graus). A orientação é comparada por matriz de rotação, e não por RPY, pelo motivo que vimos.

No contêiner, com o Gazebo rodando, primeiro confira que o nome não colide com nada seu:

```bash
ls /root/catkin_ws/src/my_workspace/tests
```

Se já existir um arquivo `fk_urdf_vs_tf.py`, **pare e me avise**. Se não, crie o script (tudo de uma vez):

```bash
cat > /root/catkin_ws/src/my_workspace/tests/fk_urdf_vs_tf.py <<'EOF'
#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

MODEL = 'j2s6s300'
BASE = MODEL + '_link_base'
EE = MODEL + '_end_effector'

def rot_rpy(r, p, y):
    # convenção do URDF: R = Rz(y) @ Ry(p) @ Rx(r)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def rot_axis(axis, q):
    # rotação de q radianos em torno de um eixo (fórmula de Rodrigues)
    a = np.array(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)

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

rospy.init_node('fk_urdf_vs_tf', anonymous=True)
robot = URDF.from_parameter_server('/robot_description')

# monta a cadeia de juntas do efetuador até a base e inverte
by_child = {j.child: j for j in robot.joints}
chain = []
link = EE
while link != BASE:
    j = by_child[link]
    chain.append(j)
    link = j.parent
chain.reverse()
print('Cadeia:', [j.name for j in chain])

js = rospy.wait_for_message('/' + MODEL + '/joint_states', JointState, timeout=10)
q = dict(zip(js.name, js.position))
print('q (rad):', [round(q[MODEL + '_joint_%d' % i], 5) for i in range(1, 7)])

T_fk = np.eye(4)
for j in chain:
    o = j.origin
    T_fk = T_fk @ transform(rot_rpy(*o.rpy), o.xyz)
    if j.type in ('revolute', 'continuous'):
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
EOF
```

- `cat > arquivo <<'EOF' ... EOF` grava o texto entre as marcas no arquivo. Ele aparece também no host (é a pasta compartilhada), mas pode ficar com dono `root`, como vimos.

Execute:

```bash
python3 /root/catkin_ws/src/my_workspace/tests/fk_urdf_vs_tf.py
```

- **Esperado:** a cadeia de 7 juntas, os ângulos, as duas posições e os dois erros.
- Se o erro for da ordem de **0,1 mm** e **0,05°**, ou menor, a sua composição de transformações está coerente com o modelo. Um erro maior indicaria um problema real, como a ordem de multiplicação ou uma convenção, e investigamos **antes** de mexer em qualquer equação.
- Se aparecer um erro em vermelho, cole o texto completo.

### Resultado

```bash
root@asusbook:~/catkin_ws# ls
build  devel  src
root@asusbook:~/catkin_ws# python3 /root/catkin_ws/src/my_workspace/tests/fk_urdf_vs_tf.py
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_1_trans']/actuator[@name='j2s6s300_joint_1_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_2_trans']/actuator[@name='j2s6s300_joint_2_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_3_trans']/actuator[@name='j2s6s300_joint_3_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_4_trans']/actuator[@name='j2s6s300_joint_4_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_5_trans']/actuator[@name='j2s6s300_joint_5_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_6_trans']/actuator[@name='j2s6s300_joint_6_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_finger_1_trans']/actuator[@name='j2s6s300_joint_finger_1_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_finger_tip_1_trans']/actuator[@name='j2s6s300_joint_finger_tip_1_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_finger_2_trans']/actuator[@name='j2s6s300_joint_finger_2_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_finger_tip_2_trans']/actuator[@name='j2s6s300_joint_finger_tip_2_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_finger_3_trans']/actuator[@name='j2s6s300_joint_finger_3_actuator']
Unknown tag "hardwareInterface" in /robot[@name='j2s6s300']/transmission[@name='j2s6s300_joint_finger_tip_3_trans']/actuator[@name='j2s6s300_joint_finger_tip_3_actuator']

Cadeia: ['j2s6s300_joint_1', 'j2s6s300_joint_2', 'j2s6s300_joint_3', 'j2s6s300_joint_4', 'j2s6s300_joint_5', 'j2s6s300_joint_6', 'j2s6s300_joint_end_effector']
q (rad): [6e-05, 2.90059, 1.2988, -2.08653, 1.4011, -2e-05]
Posição FK (m): [-0.16454  0.21636  0.53725]
Posição TF (m): [-0.16458  0.21634  0.53724]
Erro de posição (mm): 0.0473
Erro de orientação (graus): 0.0103
``` 

## Leitura dos números

| Medida | Valor |
|---|---:|
| Erro de posição | **0,047 mm** |
| Erro de orientação | **0,010°** |

Os dois ficaram abaixo da tolerância que combinamos (0,1 mm e 0,05°). Isso indica que a sua composição de transformações, feita só com os números do URDF, **reproduz o TF** nesta configuração.

**Por que não deu zero?** Hipótese: o script lê o `joint_states` e o TF em instantes ligeiramente diferentes, e o robô oscila um pouco (vimos velocidades de até ~0,1 rad/s). Essa ordem de grandeza explicaria os 0,01°. Não está provado, mas o tamanho do erro é compatível.

**Uma correção minha:** eu escrevi que a cadeia teria “9 juntas”. A saída mostra **7** (as 6 rotativas mais a fixa do efetuador), porque o script para no `link_base`. As fixas `world → root → link_base` ficam fora da cadeia.

**Sobre a pergunta pendente** (comprimentos do braço): no URDF, o “braço” é a distância da `joint_3`, `xyz = [0, -0.41, 0]`, ou seja **0,41 m**, e o “antebraço” é a da `joint_4`, `xyz = [0, 0.2073, -0.0114]`, ou seja **~0,207 m**. Eu li o sinal do eixo y como direção, não como “comprimento negativo”: o que importa é o módulo.

### O que isto ainda NÃO prova

Neste teste as juntas 1 e 6 estavam em ≈0 rad. Com ângulo zero, um erro de **sinal** ou de **ordem** nessas duas juntas não aparece: o resultado sairia igual. Então a validação das juntas 2 a 5 é razoável, e a das juntas **1 e 6** ainda não existe. Precisamos de configurações com **todas** as juntas fora de zero. Isso exige mover o robô, que é a próxima etapa.