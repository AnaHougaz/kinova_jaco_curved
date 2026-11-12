# Do zero a um robô qualquer: como descobrir tudo e validar FK/IK
 
Este guia conta o caminho, na ordem em que ele acontece, para pegar um robô que você nunca viu (um braço de outro fabricante, por exemplo) e chegar a validar a cinemática direta e inversa. Onde fez sentido, mostro como foi no seu Jaco, só como exemplo. O resto vale para qualquer robô que use ROS.
 
A ideia central é esta: **você nunca precisa adivinhar um nome**. Os nomes (tópicos, juntas, frames, controladores) estão em dois lugares, e cada um tem seus comandos:
 
- **Nos arquivos do repositório do robô** (URDF, launch, YAML): você descobre lendo, sem rodar nada.
- **No sistema rodando** (Gazebo ligado): você descobre perguntando ao ROS com `rostopic`, `rosnode`, `rosservice`, `rosparam`.
Quando um nome não é o que você esperava, o ROS sempre tem um comando que lista as opções.
 
---
 
## 1. Primeiro: achar o repositório certo e a versão certa do ROS
 
Quase todo fabricante publica o software do robô no GitHub. Procure por `<fabricante> ros` ou `<modelo> ros`. Exemplos de nomes de repositório: `kinova-ros` (Jaco), `universal_robot` (UR), `franka_ros` (Franka). A wiki do ROS (wiki.ros.org) e o index.ros.org também listam os pacotes.
 
Antes de clonar, leia o **README** e descubra três coisas:
 
1. **Qual versão do ROS** o repositório suporta (Noetic? Melodic? ROS 2?). Isso decide a imagem do Docker.
2. **Qual branch ou tag** usar. Às vezes a branch padrão é de outra versão do ROS.
3. **Quais modelos** o repositório cobre e como o nome do modelo é escrito (no Jaco, `j2s6s300`: o nome carrega o número de juntas e o tipo de punho).
Dentro do repositório, quase sempre existe um padrão de pastas. Procure por pacotes com estes nomes, nesta ordem de importância:
 
| Pasta/pacote | O que tem lá |
|---|---|
| `*_description` | O robô em si: URDF/xacro, meshes. **É a fonte da verdade da cinemática.** |
| `*_gazebo` | Launch files para simular no Gazebo |
| `*_moveit_config` | Configuração do MoveIt: grupos de planejamento, solver de IK, limites |
| `*_control` ou `*_driver` | Controladores (simulação) e comunicação com o robô real |
 
No Jaco: `kinova_description`, `kinova_gazebo`, `kinova_moveit` (com `j2s6s300_moveit_config` dentro de `robot_configs`) e `kinova_driver`.
 
---
 
## 2. Montar o ambiente com Docker
 
A lógica é: o Docker dá a você um "computador de mentira" com a versão certa do ROS (Noetic, por exemplo), sem mexer no seu Ubuntu. Você escolhe a imagem de acordo com o passo anterior.
 
Você pode partir de uma imagem pronta do ROS (por exemplo `osrf/ros:noetic-desktop-full`) e instalar MoveIt e as dependências do Gazebo por cima. O ponto importante é combinar a versão da imagem com a do repositório.
 
**No host** (Ubuntu normal), os comandos de ciclo de vida do contêiner:
 
```bash
docker build -t meu_robo_img .            # constrói a imagem a partir de um Dockerfile
docker ps -a                              # lista os contêineres (rodando ou não)
docker start -ai nome_do_conteiner        # liga um contêiner existente e entra nele
docker exec -it nome_do_conteiner bash    # abre OUTRO terminal dentro de um contêiner já ligado
```
 
E a criação do contêiner (só uma vez), com o que o Gazebo precisa para abrir janela. Cada peça tem um motivo:
 
```bash
xhost +local:docker                       # no host: libera o contêiner a desenhar janelas na sua tela
 
docker run -it --name nome_do_conteiner \
  -e DISPLAY=$DISPLAY \                   # diz onde está a tela
  -v /tmp/.X11-unix:/tmp/.X11-unix \      # compartilha o "canal" gráfico do host
  -v ~/projects/LaR/robo_novo:/root/catkin_ws/src/my_workspace \   # pasta do host <-> contêiner
  -e LIBGL_ALWAYS_SOFTWARE=1 \            # renderização por software (seu notebook com GPU híbrida precisa)
  --device=/dev/dri:/dev/dri \            # acesso à placa de vídeo
  meu_robo_img bash
```
 
Duas regras que você já aprendeu na prática: o `xhost` precisa ser repetido a cada sessão do host, e o comando `docker` só existe no host, nunca dentro do contêiner.
 
A pasta montada com `-v` é o que faz seus arquivos sobreviverem quando o contêiner é recriado.
 
---
 
## 3. Montar o workspace e compilar
 
Dentro do contêiner, o ROS 1 organiza o código em um *workspace* (`catkin_ws`). Você clona o repositório do robô dentro de `src` e compila.
 
```bash
cd ~/catkin_ws/src
git clone <url-do-repositorio-do-robo>
 
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y   # instala dependências que o repositório declara
catkin_make                                          # compila (alguns repositórios pedem `catkin build`; o README diz)
source devel/setup.bash                              # "ativa" o workspace neste terminal
```
 
O `source devel/setup.bash` vale só para o terminal onde você rodou. Em cada terminal novo (por exemplo um aberto com `docker exec`), é preciso repetir:
 
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
 
Se um comando do ROS diz que não achou um pacote, quase sempre faltou esse `source`.
 
---
 
## 4. Descobrir pelos arquivos, antes de rodar nada
 
Aqui você ainda não ligou o simulador. Só está lendo.
 
**Onde está cada pacote?**
 
```bash
rospack find nome_do_pacote           # imprime o caminho do pacote
rospack list | grep -i nome_do_robo   # lista todos os pacotes cujo nome combina
roscd nome_do_pacote                  # entra na pasta do pacote
```
 
No Jaco, `rospack find kinova_description` confirmou que o pacote estava acessível.
 
**Qual é a estrutura do robô?** Entre em `urdf/` do pacote `*_description`. Se o arquivo for `.xacro`, ele é um "URDF com macros" e precisa ser expandido primeiro:
 
```bash
xacro modelo.xacro > /tmp/robo.urdf   # expande o xacro em URDF puro
check_urdf /tmp/robo.urdf             # imprime a árvore de elos e juntas
```
 
O `check_urdf` mostra o **elo raiz** (a base) e quem está pendurado em quem. Se o comando não existir, instale `liburdfdom-tools`. Esse comando responde três perguntas ao mesmo tempo: quantas juntas o robô tem, qual é a base e qual é a ponta da cadeia.
 
Para ver detalhes de cada junta, procure no URDF:
 
```bash
grep -n "<joint " /tmp/robo.urdf      # lista as juntas e seus tipos
```
 
Dentro de cada `<joint>` você encontra o que importa para FK:
 
- `type`: `revolute` (gira com limite), `continuous` (gira sem limite), `prismatic` (desliza), `fixed` (não se move, só liga dois elos).
- `<origin xyz rpy>`: onde a junta fica em relação à anterior (a geometria que o ROS usa).
- `<axis xyz>`: em torno de qual eixo ela gira.
- `<limit>`: ângulo mínimo, máximo e velocidade.
Só as juntas que **não** são `fixed` contam como graus de liberdade. Os dedos da garra também aparecem aqui, mas não fazem parte do braço.
 
**Como se lança a simulação?** Leia a pasta `launch/` de `*_gazebo` e `*_moveit_config`. Abra o `.launch` e procure `<arg name="...">`: esses são os parâmetros que você passa na linha de comando. O ROS também lista:
 
```bash
roslaunch --ros-args nome_do_pacote arquivo.launch   # mostra os argumentos aceitos e seus valores padrão
```
 
Foi assim que apareceu `kinova_robotType:=j2s6s300` no Jaco: é um argumento do launch que escolhe o modelo.
 
**Como o MoveIt enxerga o robô?** No `*_moveit_config/config/`:
 
- `*.srdf`: define os **grupos de planejamento** (por exemplo `arm`) e qual é o elo da ponta. O nome do grupo é o que você usa nos comandos do MoveIt.
- `kinematics.yaml`: diz qual solver de IK o grupo usa (normalmente KDL).
- `joint_limits.yaml`: limites usados pelo planejador.
**Quais controladores existem?** Procure arquivos YAML de controladores (algo como `controllers.yaml` em `config/`). Eles dizem o **nome** de cada controlador, o **tipo** e **quais juntas** ele controla. O nome do controlador vira parte do nome do tópico de comando.
 
---
 
## 5. Descobrir no sistema rodando
 
Agora ligue a simulação (usando o launch que você descobriu) em um terminal. Abra um **segundo** terminal no mesmo contêiner (`docker exec -it ...`), dê o `source` nos dois arquivos de setup e pergunte ao ROS o que está rodando.
 
```bash
rosnode list          # quais programas (nós) estão ativos
rostopic list         # quais canais existem
rosservice list       # quais serviços (chamadas sob demanda) existem
rosparam list         # quais parâmetros estão carregados
```
 
### 5.1 O tópico dos estados das juntas
 
Por convenção do ROS, o estado das juntas é publicado em `/joint_states`. Mas quando o robô tem **namespace** (um prefixo com o nome do robô), vira `/nome_do_robo/joint_states`. No Jaco é `/j2s6s300/joint_states`.
 
Se o nome não é o que você esperava, você **não precisa adivinhar**:
 
```bash
rostopic list | grep -i joint           # filtra por "joint"
rostopic list | grep -i state           # ou por "state"
rostopic find sensor_msgs/JointState    # acha todos os tópicos que carregam esse TIPO de mensagem
```
 
O último é o mais seguro: o nome do tópico muda, mas o **tipo** da mensagem (`sensor_msgs/JointState`) é sempre o mesmo.
 
Achado o tópico, confirme o tipo e veja a estrutura da mensagem:
 
```bash
rostopic type /nome_do_robo/joint_states     # imprime o tipo da mensagem
rosmsg show sensor_msgs/JointState           # mostra os campos: name, position, velocity, effort
```
 
### 5.2 Ler um campo específico (é daqui que sai o `/velocity`)
 
Uma mensagem tem vários campos. Você pode ler a mensagem inteira ou só um campo, colocando o nome do campo no final do tópico:
 
```bash
rostopic echo -n 1 /nome_do_robo/joint_states            # uma mensagem completa
rostopic echo -n 1 /nome_do_robo/joint_states/name       # só os nomes (define a ORDEM dos outros campos)
rostopic echo -n 1 /nome_do_robo/joint_states/position   # só os ângulos (rad)
rostopic echo -n 1 /nome_do_robo/joint_states/velocity   # só as velocidades (rad/s)
rostopic echo -n 1 /nome_do_robo/joint_states/effort     # só os torques (N·m)
```
 
O `-n 1` pede uma única mensagem, senão o terminal fica imprimindo sem parar (interrompa com `Ctrl+C`). Os nomes dos campos (`name`, `position`, `velocity`, `effort`) são os mesmos para **qualquer robô**, porque vêm do tipo `sensor_msgs/JointState` e não do robô.
 
Detalhe importante: a posição `i` da lista `position` corresponde ao nome na posição `i` da lista `name`. Sempre leia `name` primeiro para saber qual valor é de qual junta.
 
Para saber **quem** publica o tópico e com que frequência:
 
```bash
rostopic info /nome_do_robo/joint_states   # tipo, publicadores e assinantes
rostopic hz /nome_do_robo/joint_states     # taxa de publicação
```
 
### 5.3 O TF (onde a ponta do robô está, segundo o ROS)
 
O TF é a árvore de transformações entre os elos. Para saber quais frames existem:
 
```bash
rosrun tf view_frames        # gera um frames.pdf com a árvore
rosrun rqt_tf_tree rqt_tf_tree   # ou a mesma árvore em janela, se o rqt estiver instalado
```
 
Os nomes dos frames costumam ser os nomes dos elos do URDF (os que o `check_urdf` mostrou). Escolha a **base** (a raiz) e a **ponta** (o último elo da cadeia) e peça a transformação entre eles:
 
```bash
rosrun tf tf_echo frame_da_base frame_da_ponta
```
 
Isso imprime a posição (x, y, z) e a orientação da ponta em relação à base. No Jaco, foi `link_base → end_effector`. Essa é a resposta "oficial" do ROS e é a sua **referência** (ground truth) nas comparações.
 
### 5.4 Como mover o robô (controladores e tópico de comando)
 
Primeiro descubra quais controladores estão rodando:
 
```bash
rosservice call /nome_do_robo/controller_manager/list_controllers
```
 
Cada controlador tem um nome, um tipo e um estado (`running` ou não). Depois procure o tópico de comando dele:
 
```bash
rostopic list | grep -i command
rostopic list | grep -i trajectory
rostopic type /nome_do_robo/nome_do_controlador/command
```
 
Se o tipo for `trajectory_msgs/JointTrajectory`, você manda o braço se mover publicando uma mensagem desse tipo. Para ver o que precisa preencher:
 
```bash
rosmsg show trajectory_msgs/JointTrajectory
```
 
Muitos controladores também aceitam o comando por **action** (tópicos terminados em `follow_joint_trajectory/goal`), e é o caminho que o MoveIt usa por baixo dos panos. Você pode ver isso listando os tópicos e procurando `follow_joint_trajectory`.
 
### 5.5 Olhando o conjunto
 
```bash
rosrun rqt_graph rqt_graph     # desenha quem publica e quem assina cada tópico
rosnode info /nome_do_no       # o que um nó específico publica, assina e oferece
rosparam list | grep -i description   # acha onde o URDF está carregado (robot_description)
```
 
E o RViz, para **ver** o que você está lendo:
 
```bash
rosrun rviz rviz
```
 
Dentro do RViz, ajuste o **Fixed Frame** para a base do robô e adicione os displays `RobotModel` e `TF`. Se o robô não aparece, quase sempre é o Fixed Frame errado.
 
---
 
## 6. O roteiro de validação de FK e IK, passo a passo
 
Aqui você junta tudo. A lógica geral: você tem **três** fontes de FK e precisa mostrar que elas concordam, e depois fazer o mesmo com a IK.
 
**Passo 1 — Defina a cadeia.** Pelo `check_urdf`, anote: elo da base, elo da ponta, número de juntas móveis e a ordem delas. Essa cadeia é o que você valida, e deve ser a mesma nas três fontes.
 
**Passo 2 — Reúna os parâmetros do seu modelo.** O modelo "de papel" é o que o artigo ou o manual dá (por exemplo, a tabela DH). O modelo "do ROS" é o URDF. São duas descrições do mesmo robô e podem divergir.
 
**Passo 3 — Suba a simulação e confirme o básico.** Ache o `joint_states`, leia `name` e confirme que a ordem e a quantidade de juntas batem com a cadeia do passo 1.
 
**Passo 4 — Compare (b) com o TF (URDF vs TF).** Calcule a FK compondo as transformações do URDF e compare com o `tf_echo` da ponta. Se concordam, seu código de composição está certo. Aqui o TF e o seu cálculo vêm do mesmo URDF, então isso valida o código, não o modelo.
 
**Passo 5 — Varie as poses.** Mova o robô para várias configurações (publicando no controlador, ou pelo MoveIt) e repita a comparação em cada uma. Uma pose só não prova nada, porque erros de sinal ou de eixo podem passar despercebidos quando alguns ângulos são zero. Use poses variadas, com todas as juntas fora do zero.
 
**Passo 6 — Compare (a) o seu DH com o TF.** Calcule a FK pelos parâmetros DH do artigo com os mesmos ângulos e compare com o TF. É aqui que o modelo do artigo é validado de verdade. Cuidados comuns:
 
- O **zero das juntas** do DH pode ser diferente do zero do URDF. Aparece como um offset constante nos ângulos.
- **DH padrão vs modificado** muda a ordem das transformações.
- **Unidades**: o artigo pode estar em mm, o ROS usa metros.
- A ponta do artigo pode ser o **flange**, enquanto o frame do ROS é o **centro da garra** (uma diferença fixa no eixo Z da ponta).
**Passo 7 — Compare (c) com a FK do KDL no MoveIt.** O MoveIt expõe a FK e a IK como serviços. Descubra os nomes:
 
```bash
rosservice list | grep -i compute
rosservice type /compute_fk
rossrv show moveit_msgs/GetPositionFK
```
 
Os serviços costumam ser `/compute_fk` e `/compute_ik`. O `rossrv show` mostra o que você precisa preencher (nome do grupo, juntas e ângulos). Também dá para chamá-los pelo Python com `moveit_commander`. Qualquer que seja o jeito, o grupo de planejamento vem do arquivo `.srdf`, e o solver vem do `kinematics.yaml`.
 
**Passo 8 — Valide a IK.** O truque é este: escolha uma configuração `q` conhecida, calcule a FK para obter uma pose, e então peça a IK dessa pose. Depois:
 
- **Não compare os ângulos diretamente.** Uma pose pode ter várias soluções de IK válidas (cotovelo para cima ou para baixo, punho invertido...), e o solver pode devolver outra solução que não a sua `q` original.
- **Compare as poses.** Pegue a solução de IK, passe pela FK de novo e veja se volta à pose original. O erro de posição (mm) e de orientação (graus) é a métrica.
- Preste atenção na **semente** (a configuração inicial passada ao solver numérico), porque ela influencia qual solução é encontrada.
Faça isso para sua IK e para a IK do KDL e compare as duas.
 
**Passo 9 — Registre evidências.** Em várias poses, anote erro máximo e médio de posição e orientação para cada comparação. Uma etapa só é concluída quando você tem esses números.
 
---
 
## 7. E se o robô for ROS 2, ou o nome for diferente do esperado?
 
**ROS 2.** Os conceitos são os mesmos, mas os comandos mudam de forma:
 
| ROS 1 | ROS 2 |
|---|---|
| `rostopic list` | `ros2 topic list` |
| `rostopic echo` | `ros2 topic echo` |
| `rostopic info` | `ros2 topic info` |
| `rosnode list` | `ros2 node list` |
| `rosservice list` | `ros2 service list` |
| `roslaunch pacote arq.launch` | `ros2 launch pacote arq.launch.py` |
| `rospack find` | `ros2 pkg prefix` |
 
O tipo `sensor_msgs/JointState` continua existindo, então a busca pelo tipo continua sendo o jeito mais seguro de achar o tópico.
 
**Quando um nome não bate com o esperado**, o procedimento é sempre o mesmo:
 
1. Liste (`rostopic list`, `rosnode list`, `rosservice list`, `rosparam list`).
2. Filtre com `grep -i`.
3. Se ainda não achar, procure pelo **tipo** (`rostopic find`) ou leia o arquivo de launch e de configuração do pacote.
4. Confirme com `type`, `info` e `echo -n 1` antes de usar.
---
 
## 8. Mapa rápido: "tenho esta pergunta, uso este comando"
 
- Onde está o pacote? → `rospack find`
- Quantas juntas o robô tem, qual é a base e a ponta? → `xacro` + `check_urdf`
- Quais argumentos o launch aceita? → `roslaunch --ros-args`
- Qual é o nome do tópico de estado das juntas? → `rostopic find sensor_msgs/JointState`
- Que campos tem essa mensagem? → `rosmsg show`
- Qual é a ordem das juntas? → `rostopic echo -n 1 <tópico>/name`
- Quais são os ângulos, velocidades e torques? → `.../position`, `.../velocity`, `.../effort`
- Quem publica esse tópico? → `rostopic info`
- Quais frames existem? → `rosrun tf view_frames`
- Onde está a ponta do robô? → `rosrun tf tf_echo base ponta`
- Quais controladores existem e como mandar o braço mexer? → `list_controllers` + `rostopic type .../command`
- Como o MoveIt calcula FK/IK? → `rosservice list | grep compute` + `rossrv show`
- Como funciona o conjunto todo? → `rqt_graph`
