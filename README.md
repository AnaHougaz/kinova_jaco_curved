
# Projeto Jaco - Validação Cinemática e Simulação

Este repositório contém a implementação e validação da cinemática direta (FK) e inversa (IK) do robô Kinova Jaco Gen2, em ambiente simulado (Gazebo/Docker) e no hardware real.

## Sumário
- [Resultados Obtidos](#resultados-obtidos)
- [Estrutura de Branches](#estrutura-de-branches)
- [Rotina 1: Notebook (Simulação via Docker)](#rotina-1-notebook-simulação-via-docker)
- [Rotina 2: Laboratório (Hardware Real)](#rotina-2-laboratório-hardware-real)
- [Solução de Problemas (Troubleshooting)](#solução-de-problemas-troubleshooting)

---

## Resultados Obtidos

| Configuração | Modelo | Erro FK | Erro IK |
|---|---|---|---|
| Simulação | `j2s6s300` (esférico) | 0.085 mm | 0.070 mm |
| Simulação | `j2n6s300` (curvo) | 0.070 mm | 0.067 mm |
| Hardware | `j2n6s300` (curvo) | 0.076 mm | 0.076 mm |

---

## Estrutura de Branches

O projeto foi dividido para manter a organização entre o que roda no simulador e o que roda no robô real.

*   **Repositório `kinova_jaco`:** Versão antiga com o modelo esférico (`j2s6s300`), usado como base inicial por causa do artigo de referência (Pieper).
*   **Repositório `kinova_jaco_curved`:** Versão atual com o modelo do laboratório (`j2n6s300`).
    *   **Branch `develop`:** Use esta branch para **Simulação** no Gazebo.
    *   **Branch `hardware-validation`:** Use esta branch para **Hardware Real** (scripts adaptados para ler os dados do robô físico).

---

## Rotina 1: Notebook (Simulação via Docker)

**Atenção:** O nome do container atual é `kinova_hardware` e ele monta a pasta do repositório curvo (`~/projects/LaR/kinova_jaco_curved`).

### 1. Iniciar o Container
No terminal do seu computador (Host), rode:
```bash
# Dar permissão para interface gráfica
xhost +local:docker

# Iniciar o container já criado (se for a primeira vez, veja o Troubleshooting)
docker start -ai kinova_hardware
```

### 2. Abrir o Gazebo
Dentro do container, execute na ordem exata:
```bash
# 1. Vá para a raiz do workspace
cd /root/catkin_ws

# 2. Compile (necessário se alterou arquivos xacro/launch)
catkin_make

# 3. Carregue o sistema ROS PRIMEIRO
source /opt/ros/noetic/setup.bash

# 4. Carregue o workspace DEPOIS
source devel/setup.bash

# 5. Rode o Gazebo
roslaunch jaco_sim jaco_realsense.launch
```

### 3. Ver a Câmera (Opcional)
Abra uma **nova aba** no seu notebook, entre no container (`docker exec -it kinova_hardware bash`) e rode:
```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rqt_image_view
```
*(Selecione o tópico `/camera/color/image_raw` na janela).*

### 4. Rodar os Testes de Simulação
Com o Gazebo aberto, abra uma **nova aba** no terminal, entre no container e rode:
```bash
docker exec -it kinova_hardware bash

cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd src/my_workspace

# Teste de FK
python3 tests/fk/fk_dh_vs_tf.py

# Teste de IK
python3 tests/ik/ik_kdl_vs_fk.py
```

---

## Rotina 2: Laboratório (Hardware Real)

**Atenção:** No laboratório, o ROS roda **nativo** (sem Docker). O modelo é o `j2n6s300`. Use a branch `hardware-validation`.

### 1. Preparar o Robô (Primeira vez)
Ligue o robô e conecte o cabo USB.
```bash
# Criar regra de permissão para o USB do robô
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="22cd", MODE="0666"' | sudo tee /etc/udev/rules.d/99-kinova.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verificar se o Linux está vendo o robô (deve aparecer Kinova)
lsusb
```

### 2. Configurar a Rede do ROS (Terminal do Lab)
O ROS precisa saber quem é o "mestre".
```bash
# Definir localhost como mestre e o IP do PC do lab
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=192.168.13.100
```

### 3. Iniciar o Driver do Robô (Terminal 1)
```bash
source ~/jaco_ws/devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300
```
*(Deixe este terminal aberto. Você deve ouvir o robô fazendo um barulho de inicialização).*

### 4. Verificar o Tópico (Terminal 2 - Opcional)
```bash
source ~/jaco_ws/devel/setup.bash
rostopic list | grep -i joint
# O tópico correto é: /j2n6s300_driver/out/joint_state
```

### 5. Rodar os Testes no Robô (Terminal 3)
Navegue até a pasta onde você baixou os scripts (ex: `~/anahougaz`) e rode:
```bash
source ~/jaco_ws/devel/setup.bash

# Validar FK no hardware
python3 tests/fk/fk_dh_vs_tf.py

# Validar IK (Mover o robô - Cuidado!)
python3 tests/ik/ik_real_test.py
```
⚠️ **Atenção:** Tenha certeza que o professor/técnico está com o botão de emergência em mãos antes de rodar o `ik_real_test.py`.

---

## Solução de Problemas (Troubleshooting)

### Como recriar o container no notebook do zero (se der erro):
Se você precisar criar o container `kinova_hardware` de novo:
```bash
xhost +local:docker
docker run -it \
  --name kinova_hardware \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/projects/LaR/kinova_jaco_curved:/root/catkin_ws/src/my_workspace" \
  --device=/dev/dri:/dev/dri \
  --network host \
  kinova-ros-noetic
```

### Erro `Multiple packages found` (jaco_sim):
Você montou a pasta do repositório antigo e do novo ao mesmo tempo. Feche e recrie o container montando **apenas** a pasta correta (`kinova_jaco_curved`).

### Erro `Permission denied` ao salvar no VS Code:
O Docker criou arquivos como `root`. No terminal do seu notebook (fora do Docker):
```bash
sudo chown -R $USER:$USER ~/projects/LaR/kinova_jaco_curved/
```

### Erro `resource not found: realsense2_description`:
O container novo não tem o pacote da câmera instalado. Rode dentro do container:
```bash
apt-get update
apt-get install -y ros-noetic-realsense2-description
```

### Erro `No module named 'kdl_parser_py'`:
Rode dentro do container:
```bash
apt-get install -y ros-noetic-kdl-parser-py
```