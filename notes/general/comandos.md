# Glossário do projeto LaR — Kinova Jaco (`j2s6s300`)

Pasta do projeto: `~/projects/LaR/jaco/` (host) = `/root/catkin_ws/src/my_workspace` (contêiner `kinova_dev`).

> **Namespace:** o prefixo dos tópicos muda com o modelo. Com `j2s6s300` é `/j2s6s300/...`; com `j2n6s300` seria `/j2n6s300/...`.

---

## 1. Conceitos básicos

| Termo | O que é | Exemplo no Jaco |
|---|---|---|
| **ROS** | Sistema que faz programas do robô conversarem entre si | ROS Noetic, dentro do contêiner |
| **Nó (node)** | Um programa dentro do ROS | O Gazebo, o controlador, o MoveIt |
| **Tópico (topic)** | "Canal" por onde os nós publicam ou recebem mensagens | `/j2s6s300/joint_states` |
| **Mensagem** | O formato do dado que passa no tópico | `sensor_msgs/JointState` |
| **Publicar / assinar** | Enviar / receber mensagens num tópico | `rostopic pub` publica; `rostopic echo` assina |
| **URDF / xacro** | Arquivo que descreve o robô (elos, juntas, eixos, medidas) | `kinova_description/urdf/j2s6s300.xacro` |
| **Junta (joint)** | Articulação do braço | `j2s6s300_joint_1` … `joint_6` |
| **Elo (link)** | Peça rígida entre duas juntas | `j2s6s300_link_base` |
| **TF** | TF do ROS: o computador multiplicando A1·…·A6 em tempo real, mas usando o URDF, não a DH | `link_base → end_effector` |
| **Controlador** | Nó que faz as juntas seguirem a posição pedida | `effort_joint_trajectory_controller` |
| **Gazebo** | Simulador físico (gravidade, contato, motores) | `gzserver` (física) + `gzclient` (janela) |
| **RViz** | Visualizador: mostra TF, modelo do robô e o que o MoveIt planeja | — |
| **MoveIt** | Planejador de movimento, com FK/IK (solver KDL) | pacote `j2s6s300_moveit_config` |
| **KDL** | Solver de cinemática usado pelo MoveIt | referência (c) da validação |
| **FK / IK** | Cinemática direta (ângulos → pose) / inversa (pose → ângulos) | `fk_ik_jaco.py` |
| **DH** | Parâmetros Denavit-Hartenberg que descrevem a cadeia | Tabela 5 do artigo |
| **RPY** | Orientação como roll, pitch, yaw | Pose inicial ≈ [-166,7°, 60,4°, -86,9°] |

---

## 2. Tópicos e campos do estado das juntas

| Tópico / campo | O que significa | Unidade |
|---|---|---|
| `/j2s6s300/joint_states` | Estado de todas as juntas (mensagem `sensor_msgs/JointState`) | — |
| `.../joint_states/name` | Nome de cada junta (define a ordem dos outros campos) | texto |
| `.../joint_states/position` | Ângulo de cada junta (os `q1…q6`) | rad |
| `.../joint_states/velocity` | Velocidade de cada junta (parado ≈ 0, ruído de ~1e-5 é normal) | rad/s |
| `.../joint_states/effort` | Torque de cada junta | N·m |
| `/j2s6s300/effort_joint_trajectory_controller/command` | Tópico onde você **manda** a trajetória para o braço | mensagem `trajectory_msgs/JointTrajectory` |

A lista tem 9 valores: 6 juntas do braço + 3 dedos da garra (o "3" de `s300`).

---

## 3. Comandos usados

### No host (terminal normal do Ubuntu)

| Comando | Para que serve |
|---|---|
| `xhost +local:docker` (ou `xhost +SI:localuser:root`) | Libera a janela do Gazebo a aparecer a partir do contêiner. Repetir a cada sessão do host |
| `docker start -ai kinova_dev` | Liga o contêiner e entra nele |
| `docker exec -it kinova_dev bash` | Abre **outro** terminal dentro do contêiner já ligado (`docker` só existe no host) |

### No contêiner `kinova_dev`

| Comando | Para que serve |
|---|---|
| `echo $ROS_DISTRO` | Confirma a versão do ROS (esperado: `noetic`) |
| `rospack find kinova_description` | Confirma que o pacote do robô foi achado |
| `roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2s6s300` | Abre o Gazebo com o robô. Troque pelo modelo desejado (`j2n6s300` também funcionou) |
| `roslaunch j2s6s300_moveit_config j2s6s300_gazebo_demo.launch` | Abre o MoveIt com o Gazebo (usado no seu README) |
| `rostopic list` | Lista todos os tópicos ativos |
| `rostopic echo -n 1 /j2s6s300/joint_states` | Mostra **uma** mensagem completa do estado das juntas |
| `rostopic echo -n 1 /j2s6s300/joint_states/velocity` | Mostra só o campo velocidade (troque por `position`, `name`, `effort`) |
| `rostopic hz /j2s6s300/joint_states` | Mede a frequência de publicação do tópico |
| `rostopic pub -1 /j2s6s300/effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory ...` | Envia **uma vez** (`-1`) uma trajetória e move o braço |
| `python3 tests/fk_urdf_vs_tf.py` | Compara a FK montada a partir do URDF com o TF `link_base → end_effector` |
| `python3 fk_ik_jaco.py` | Roda sua FK por DH e a IK numérica (scipy `least_squares`) |

Se você leu o TF com `rosrun tf tf_echo`, o comando é `rosrun tf tf_echo j2s6s300_link_base j2s6s300_end_effector`. Ele imprime a posição (x, y, z) e a orientação do efetuador em relação à base.

---

## 4. Nomes que aparecem no robô

| Nome | O que é |
|---|---|
| `j2s6s300` | Jaco Gen2 (`j2`), **s**pherical wrist (`s`), 6 graus de liberdade (`6`), garra de 3 dedos (`3`). Punho esférico, necessário para o Pieper do artigo |
| `j2n6s300` | Mesmo robô com punho **n**ão esférico (`n`). Modelo do robô real do laboratório: **ainda não confirmado** |
| `j2s6s300_joint_1 … _6` | As 6 juntas do braço |
| `j2s6s300_link_base` | Frame da base do braço |
| `j2s6s300_end_effector` | Frame da ponta do braço |

---

## 5. Valores de referência já medidos

| Item | Valor |
|---|---|
| Pose inicial (repouso) | `q ≈ [0, 2.9006, 1.2988, -2.0865, 1.4011, 0]` rad |
| TF na pose inicial | posição ≈ `[-0.165, 0.216, 0.537]` m |
| FK (URDF) vs TF, pose inicial | erro de 0,047 mm e 0,010° |
| Alvo enviado ao controlador | `[0.5, 2.9, 1.3, -2.09, 1.4, 0.5]` rad |
| Erro do controlador nesse alvo | máx. 3,3e-3 rad (junta 4) |
| FK (URDF) vs TF, com `q1 = q6 = 0.5` | erro de 0,017 mm e 0,006° |

---

## 6. Flags e configurações que não podem faltar

| Item | Por quê |
|---|---|
| `LIBGL_ALWAYS_SOFTWARE=1` | A GPU híbrida faz o Gazebo dar segfault sem isso |
| `--device=/dev/dri:/dev/dri` | Também é necessário para o Gazebo na criação do contêiner |
| `xhost` no host | Sem ele o `gzclient` morre com "Authorization required" (exit 134) |

---

## 7. Ainda não usados, mas vão aparecer

| Item | Para que serve |
|---|---|
| `rviz` | Visualizar o robô e o TF |
| `rosnode list` | Ver os nós ativos |
| `rostopic info <tópico>` | Ver o tipo da mensagem e quem publica/assina |
| `rosmsg show trajectory_msgs/JointTrajectory` | Ver a estrutura da mensagem que você publica |
| `tests/fk_multi_config.py` | Próximo script: validar a FK em várias poses |