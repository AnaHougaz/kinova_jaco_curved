# Ler a geometria que o modelo usa (URDF)

Objetivo: ver os números que a simulação usa para calcular a FK: as 6 juntas, os eixos de rotação e as distâncias entre elas. É a base para a sua equação ser comparada com o modelo e não com o artigo. Isso corresponde ao item 3 dos objetivos.

Conceito: no URDF cada junta tem parent e child (os elos que ela liga), axis (eixo de rotação) e origin (xyz em metros e rpy em radianos: onde a junta está em relação ao elo pai). A FK é o produto dessas transformações em sequência, cada uma girada pelo ângulo da junta.

No Terminal B (dentro do contêiner), com o Gazebo rodando, cole o bloco inteiro:

```bash
python3 - <<'EOF'
from urdf_parser_py.urdf import URDF
r = URDF.from_parameter_server('/robot_description')
for j in r.joints:
    if 'finger' in j.name:
        continue
    o = j.origin
    print(j.name, j.type, j.parent, '->', j.child, 'xyz=', o.xyz if o else None, 'rpy=', o.rpy if o else None, 'axis=', j.axis)
EOF
```

- python3 - <<'EOF' ... EOF roda o texto entre as marcas como um script, sem criar arquivo.
- O script lê o modelo carregado no ROS (/robot_description, que apareceu na lista de parâmetros do lançamento) e imprime cada junta, pulando as dos dedos.
- Esperado: cerca de 8 linhas, entre juntas fixas (sem eixo) e as seis rotativas.
- Se aparecer ModuleNotFoundError: urdf_parser_py, o pacote não está instalado. Não instale nada: copie o erro e eu proponho a alternativa.

### Resultado:

```bash
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
connect_root_and_world fixed world -> root xyz= [0.0, 0.0, 0.0] rpy= [0.0, 0.0, 0.0] axis= None
j2s6s300_joint_base fixed root -> j2s6s300_link_base xyz= [0.0, 0.0, 0.0] rpy= [0.0, 0.0, 0.0] axis= [0.0, 0.0, 0.0]
j2s6s300_joint_1 continuous j2s6s300_link_base -> j2s6s300_link_1 xyz= [0.0, 0.0, 0.15675] rpy= [0.0, 3.141592653589793, 0.0] axis= [0.0, 0.0, 1.0]
j2s6s300_joint_2 revolute j2s6s300_link_1 -> j2s6s300_link_2 xyz= [0.0, 0.0016, -0.11875] rpy= [-1.5707963267948966, 0.0, 3.141592653589793] axis= [0.0, 0.0, 1.0]
j2s6s300_joint_3 revolute j2s6s300_link_2 -> j2s6s300_link_3 xyz= [0.0, -0.41, 0.0] rpy= [0.0, 3.141592653589793, 0.0] axis= [0.0, 0.0, 1.0]
j2s6s300_joint_4 continuous j2s6s300_link_3 -> j2s6s300_link_4 xyz= [0.0, 0.2073, -0.0114] rpy= [-1.5707963267948966, 0.0, 3.141592653589793] axis= [0.0, 0.0, 1.0]
j2s6s300_joint_5 revolute j2s6s300_link_4 -> j2s6s300_link_5 xyz= [0.0, 0.0, -0.10375] rpy= [1.5707963267948966, 0.0, 3.141592653589793] axis= [0.0, 0.0, 1.0]
j2s6s300_joint_6 continuous j2s6s300_link_5 -> j2s6s300_link_6 xyz= [0.0, 0.10375, 0.0] rpy= [-1.5707963267948966, 0.0, 3.141592653589793] axis= [0.0, 0.0, 1.0]
j2s6s300_joint_end_effector fixed j2s6s300_link_6 -> j2s6s300_end_effector xyz= [0.0, 0.0, -0.16] rpy= [3.141592653589793, 0.0, 1.5707963267948966] axis= [0.0, 0.0, 0.0]
```
### O que a saída mostra

Os avisos Unknown tag "hardwareInterface" vêm da biblioteca de leitura, que não conhece essa tag das transmissões do Gazebo. Hipótese: não afetam a geometria, porque as linhas das juntas saíram completas.

### Sobre as juntas:

- Fixas (sem eixo): connect_root_and_world, j2s6s300_joint_base e j2s6s300_joint_end_effector.
- Rotativas (eixo [0,0,1]): joint_1 a joint_6. Três são continuous (1, 4 e 6, sem limite de ângulo) e três são revolute (2, 3 e 5, com limites).

O que isso significa para a FK:

- world → root → link_base são transformações identidade. O mundo, a raiz e a base do braço coincidem, e os dois referenciais da comparação estão alinhados.
- A transformação final link_6 → end_effector não é nula: xyz = [0, 0, -0.16] e rpy = [π, 0, π/2]. Uma equação que pare no punho não vai coincidir com o end_effector do TF.
- O eixo de cada junta é [0,0,1] no referencial do elo filho. Como a origem da joint_1 tem rpy = [0, π, 0], o eixo z do link_1 aponta para −z da base. Isso é derivado, não testado: um ângulo positivo na junta 1 gira no sentido oposto ao que um desenho com z para cima sugeriria. Sinais de ângulo são uma fonte clássica de diferença entre equações de artigo e o modelo.
- Os comprimentos aparecem como xyz: 0.41 (junta 3), 0.2073 (junta 4) e 0.10375 (juntas 5 e 6).


