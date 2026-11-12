Aqui está o conteúdo em **Markdown (`.md`)**, organizado em um único bloco `bash` para você copiar e colar diretamente no seu documento:

```bash
# Teste de FK automatizado em várias configurações

## Objetivo

Repetir o procedimento em 4 configurações diferentes, com o script movendo o robô, esperando, lendo e comparando os resultados, além de salvar uma tabela em arquivo como evidência para o professor.

Isso proporciona a reprodutibilidade desejada.

## O que o script faz

- **Verifica os limites:** lê os limites das juntas rotativas no URDF antes de mover o robô. Se algum alvo estiver fora dos limites, aborta sem movimentar o braço.
- **Move gradualmente:** utiliza 8 segundos por movimento, com deslocamentos de até aproximadamente 0,5 rad por junta.
- **Compara a FK com o TF de duas formas:**
  - Com o último TF disponível.
  - Com o TF no mesmo instante do `joint_states`.

  Essa comparação testa a hipótese de desalinhamento temporal: se o erro diminuir bastante na segunda forma, a hipótese ganha sustentação.
- **Retorna à pose inicial:** ao final, o script tenta retornar o robô à configuração inicial definida em `HOME`.
- **Salva os resultados:** gera um arquivo CSV com os resultados das quatro configurações.

## 1. Verificar se o arquivo já existe

**Local:** Terminal B, dentro do contêiner, com o Gazebo em execução.

Primeiro, confirme se o nome do arquivo está livre:

```bash
test -e /root/catkin_ws/src/my_workspace/tests/fk_multi_config.py && echo EXISTE || echo livre
```

- Se aparecer `EXISTE`, pare e verifique o arquivo existente.
- Se aparecer `livre`, crie o script na etapa seguinte.

## 2. Criar o script de teste

Execute o comando abaixo no Terminal B:

```bash
cat > /root/catkin_ws/src/my_workspace/tests/fk_multi_config.py <<'EOF'
#!/usr/bin/env python3
import csv
import sys
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF

MODEL = 'j2s6s300'
BASE = MODEL + '_link_base'
EE = MODEL + '_end_effector'
JOINTS = [MODEL + '_joint_%d' % i for i in range(1, 7)]
CMD = '/%s/effort_joint_trajectory_controller/command' % MODEL
HOME = [0.0, 2.9, 1.3, -2.09, 1.4, 0.0]
CONFIGS = [
    [-0.8, 2.65, 1.05, -1.85, 1.25, -1.0],
    [1.2, 3.15, 1.55, -2.35, 1.55, 1.5],
    [-1.5, 2.9, 1.3, -2.09, 1.4, 2.5],
    [2.0, 2.75, 1.1, -1.9, 1.3, -2.0],
]
MOVE_TIME = 8.0
SETTLE = 6.0
OUT = '/root/catkin_ws/src/my_workspace/tests/fk_results.csv'

def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def rot_axis(axis, q):
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

def fk(chain, qd):
    T = np.eye(4)
    for j in chain:
        o = j.origin
        T = T @ transform(rot_rpy(*o.rpy), o.xyz)
        if j.type in ('revolute', 'continuous'):
            T = T @ transform(rot_axis(j.axis, qd[j.name]), [0, 0, 0])
    return T

def tf_pose(buf, stamp):
    tr = buf.lookup_transform(BASE, EE, stamp, rospy.Duration(2.0))
    t, r = tr.transform.translation, tr.transform.rotation
    return transform(quat_to_rot(r.x, r.y, r.z, r.w), [t.x, t.y, t.z])

def errors(T1, T2):
    dp = np.linalg.norm(T1[:3, 3] - T2[:3, 3]) * 1000.0
    c = (np.trace(T1[:3, :3].T @ T2[:3, :3]) - 1) / 2
    da = np.degrees(np.arccos(np.clip(c, -1, 1)))
    return dp, da

def read_q():
    js = rospy.wait_for_message('/%s/joint_states' % MODEL, JointState, timeout=10)
    return js, dict(zip(js.name, js.position))

def move(pub, target):
    msg = JointTrajectory()
    msg.joint_names = JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = list(target)
    pt.time_from_start = rospy.Duration(MOVE_TIME)
    msg.points = [pt]
    pub.publish(msg)
    rospy.sleep(MOVE_TIME + SETTLE)

rospy.init_node('fk_multi_config', anonymous=True)
robot = URDF.from_parameter_server('/robot_description')
by_child = {j.child: j for j in robot.joints}
chain = []
link = EE
while link != BASE:
    j = by_child[link]
    chain.append(j)
    link = j.parent
chain.reverse()

lims = {j.name: (j.limit.lower, j.limit.upper) for j in robot.joints
        if j.type == 'revolute' and j.limit is not None}
print('Limites (rad) das juntas rotativas com limite:')
for k, v in lims.items():
    print('  %s: [%.3f, %.3f]' % (k, v[0], v[1]))
for cfg in CONFIGS + [HOME]:
    for n, v in zip(JOINTS, cfg):
        if n in lims and not (lims[n][0] <= v <= lims[n][1]):
            print('ABORTADO sem mover: %s=%.3f fora do limite %s' % (n, v, lims[n]))
            sys.exit(1)

pub = rospy.Publisher(CMD, JointTrajectory, queue_size=1)
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf)
while pub.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)
rospy.sleep(1.0)

rows = []
for k, cfg in enumerate(CONFIGS, 1):
    print('\n--- Configuração %d: %s' % (k, cfg))
    move(pub, cfg)
    js, qd = read_q()
    q = np.array([qd[n] for n in JOINTS])
    jerr = float(np.max(np.abs(q - np.array(cfg))))
    T_fk = fk(chain, qd)
    dp0, da0 = errors(T_fk, tf_pose(buf, rospy.Time(0)))
    try:
        dp1, da1 = errors(T_fk, tf_pose(buf, js.header.stamp))
    except Exception as e:
        dp1, da1 = float('nan'), float('nan')
        print('TF alinhado indisponível:', e)
    ok = jerr < 0.02 and dp1 < 0.1 and da1 < 0.05
    print('q lido (rad):', np.round(q, 4))
    print('erro máx. de junta (rad): %.4f' % jerr)
    print('FK vs TF (último):    %.4f mm  %.4f graus' % (dp0, da0))
    print('FK vs TF (alinhado):  %.4f mm  %.4f graus' % (dp1, da1))
    print('RESULTADO:', 'OK' if ok else 'FALHOU')
    rows.append([k] + list(cfg) + [jerr, dp0, da0, dp1, da1, 'OK' if ok else 'FALHOU'])

print('\nVoltando à pose inicial...')
move(pub, HOME)

with open(OUT, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['config'] + ['q%d_alvo' % i for i in range(1, 7)] +
               ['erro_junta_max_rad', 'pos_ultimo_mm', 'ori_ultimo_deg',
                'pos_alinhado_mm', 'ori_alinhado_deg', 'resultado'])
    w.writerows(rows)
print('Tabela salva em', OUT)
print('RESUMO GERAL:', 'OK' if all(r[-1] == 'OK' for r in rows) else 'HÁ FALHAS')
EOF
```

## 3. Executar o teste

O teste pode levar alguns minutos, pois cada movimento aguarda aproximadamente 14 segundos de simulação.

Execute:

```bash
python3 /root/catkin_ws/src/my_workspace/tests/fk_multi_config.py 2>&1 | grep -v hardwareInterface
```

### Resultados esperados

- Primeiro, o script apresenta os limites das juntas.
- Depois, apresenta um bloco de resultados para cada configuração.
- Ao final, tenta retornar o robô à pose inicial.
- O script salva a tabela em `fk_results.csv`.

**Atenção:** se aparecer `ABORTADO sem mover`, o script identificou um alvo fora dos limites definidos no URDF e encerrou antes de publicar os movimentos. Envie a saída para análise.

Se o robô apresentar movimento inesperado, interrompa com `Ctrl+C`. A interrupção pode impedir a execução do retorno à pose inicial; nesse caso, não presuma que o braço voltou à configuração segura.

## 4. Localização do arquivo CSV

O arquivo será salvo em:

`/root/catkin_ws/src/my_workspace/tests/fk_results.csv`

Como o script é executado dentro do contêiner, o arquivo pode pertencer ao usuário `root`, o que pode causar problemas de permissão ao editá-lo no host.

## 5. O que enviar para análise

Envie:

1. A saída completa do script.
2. Duas observações sobre o comportamento do robô e os resultados.

```bash
root@asusbook:~/catkin_ws# python3 /root/catkin_ws/src/my_workspace/tests/fk_multi_config.py 2>&1 | grep -v hardwareInterface
Limites (rad) das juntas rotativas com limite:
  j2s6s300_joint_2: [0.820, 5.463]
  j2s6s300_joint_3: [0.332, 5.952]
  j2s6s300_joint_5: [0.524, 5.760]
  j2s6s300_joint_finger_1: [0.000, 1.510]
  j2s6s300_joint_finger_tip_1: [0.000, 2.000]
  j2s6s300_joint_finger_2: [0.000, 1.510]
  j2s6s300_joint_finger_tip_2: [0.000, 2.000]
  j2s6s300_joint_finger_3: [0.000, 1.510]
  j2s6s300_joint_finger_tip_3: [0.000, 2.000]

--- Configuração 1: [-0.8, 2.65, 1.05, -1.85, 1.25, -1.0]
q lido (rad): [-7.0834  2.65    1.0488 -1.8535  1.2514  5.2832]
erro máx. de junta (rad): 6.2834
FK vs TF (último):    0.0000 mm  0.0000 graus
FK vs TF (alinhado):  0.0000 mm  0.0000 graus
RESULTADO: FALHOU

--- Configuração 2: [1.2, 3.15, 1.55, -2.35, 1.55, 1.5]
q lido (rad): [-5.0832  3.1514  1.5487 -2.3531  1.5507  7.7831]
erro máx. de junta (rad): 6.2832
FK vs TF (último):    0.0000 mm  0.0000 graus
FK vs TF (alinhado):  0.0000 mm  0.0000 graus
RESULTADO: FALHOU

--- Configuração 3: [-1.5, 2.9, 1.3, -2.09, 1.4, 2.5]
q lido (rad): [-7.7832  2.9006  1.2988 -2.0931  1.4014  8.7832]
erro máx. de junta (rad): 6.2832
FK vs TF (último):    0.0000 mm  0.0000 graus
FK vs TF (alinhado):  0.0000 mm  0.0000 graus
RESULTADO: FALHOU

--- Configuração 4: [2.0, 2.75, 1.1, -1.9, 1.3, -2.0]
q lido (rad): [-10.5664   2.7504   1.0988  -1.9029   1.3041  10.5667]
erro máx. de junta (rad): 12.5667
FK vs TF (último):    0.0000 mm  0.0000 graus
FK vs TF (alinhado):  0.0000 mm  0.0000 graus
RESULTADO: FALHOU

Voltando à pose inicial...
Tabela salva em /root/catkin_ws/src/my_workspace/tests/fk_results.csv
RESUMO GERAL: HÁ FALHAS
root@asusbook:~/catkin_ws# ls
build  devel  src
root@asusbook:~/catkin_ws# 
```