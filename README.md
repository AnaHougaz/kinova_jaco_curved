# Projeto Jaco 

| Trecho | Significado |
|:---|:---|
| `j2` | Jaco, geração 2 (Gen2) |
| `s` ou `n` | Punho **s**férico ou **n**ão esférico |
| `6` | 6 graus de liberdade |
| `s` | Versão *Service* (existe também a versão *Assistive*) |
| `300` | Gripper de 3 dedos |

### Estamos usando o ```j2n6s300```

### Pose HOME ```[0.0, 2.9, 1.3, -2.07, 1.4, 0.0]```

## Rodar o Gazebo

```bash 
cd /root/catkin_ws
catkin_make
source devel/setup.bash
roslaunch jaco_sim jaco_realsense.launch
```
## Rodar a camera

```bash 
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rostopic list | grep camera
```
```bash 
rqt_image_view
```

## 1. Docker

O ROS Noetic exige Ubuntu 20.04, mas o host roda Ubuntu 26.04. Em vez de
fazer dual-boot ou downgrade, isolamos o ROS Noetic dentro de um
container Docker — o host nunca é modificado, e drivers gráficos
problemáticos (GPU híbrida Intel + NVIDIA) ficam isolados também.

## 2. Instalação do ambiente (feita uma única vez)

```bash
sudo apt update
```

```bash
sudo apt install nvidia-driver-libs:i386
```

### 2.1. Instalar o Docker Engine

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```
(fazer logout/login depois)

Se aparecer erro de dpkg travado durante instalação de outros pacotes:
```bash
sudo dpkg --configure -a
sudo apt-get update
```

### 2.2. Estrutura de pastas

```
~/projects/LaR/jaco/          <- pasta do projeto (montada como volume no container)
├── Dockerfile
├── README.md                 <- este arquivo
├── fk_ik_jaco.py              <- script de FK/IK
└── (dentro do container, aparece em /root/catkin_ws/src/my_workspace)
```

### 2.3. Dockerfile

```dockerfile
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-pip \
    ros-noetic-moveit \
    ros-noetic-moveit-visual-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/Kinovarobotics/kinova-ros.git
WORKDIR /root/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
```

### 2.4. Build da imagem (uma vez)

```bash
cd ~/projects/LaR/jaco
docker build -t kinova-ros-noetic .
```

### 2.5. Primeira execução do container

```bash
xhost +local:docker
```

```bash
docker rm kinova_dev
```

```bash
docker run -it \
  --name kinova_dev \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/projects/LaR/jaco:/root/catkin_ws/src/my_workspace" \
  --device=/dev/dri:/dev/dri \
  --network host \
  kinova-ros-noetic
```

Notas sobre as flags:
- `DISPLAY` + volume do `/tmp/.X11-unix` → permite que janelas gráficas
  (Gazebo, RViz) do container apareçam na tela do host, via X11.
- `LIBGL_ALWAYS_SOFTWARE=1` + `--device=/dev/dri` → renderização 3D via
  CPU (Mesa/llvmpipe) em vez da GPU, necessário porque a GPU híbrida
  (Intel + NVIDIA RTX 2050) causava `Segmentation fault` no Gazebo.
- `--volume="$HOME/projects/LaR/jaco:..."` → tudo que salvamos dentro
  dessa pasta no container sobrevive no disco real do host.

## 3. Rotina diária (depois que o container já existe)

Não repita o `docker build` nem o `docker run` completo depois da
primeira vez — isso recria tudo do zero. Use:

```bash
xhost +local:docker
```

```bash
docker start -ai kinova_dev
```

Isso religa o mesmo container, com tudo que foi instalado nele preservado.

### 3.1. Abrir o Gazebo (simulação física do Jaco)

Dentro do container:
```bash
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2s6s300
```

> Usamos `j2s6s300` (punho **esférico**), não `j2n6s300`, porque o
> algoritmo de IK do artigo (Pieper) só é válido para punho esférico.

### 3.2. Abrir o MoveIt + RViz (planejamento de movimento)

Em um **segundo terminal**:
```bash
docker exec -it kinova_dev bash
roslaunch j2s6s300_moveit_config j2s6s300_gazebo_demo.launch
```

# Ativando o venv
``` bash
source .venv/bin/activate
```

# Executando scripts Python dentro do Docker

### 1. Acessar o container Docker

Após entrar no container, o terminal apresentará um prompt semelhante a:

```bash
root@asusbook:~/catkin_ws#
```

### 2. Acessar o diretório do projeto

```bash
cd /root/catkin_ws/src/my_workspace
```

### 3. Carregar o ambiente ROS Noetic

```bash
source /opt/ros/noetic/setup.bash
```

### 4. Executar o script Python

```bash
python3 nome_do_script.py
```

## Instalando e verificando o ```kdl_parser_py```no Docker

### 1. Atualizar a lista de pacotes

Dentro do container Docker, execute:

```bash
apt-get update
```

### 2. Instalar o pacote ROS Noetic

```bash
apt-get install -y ros-noetic-kdl-parser-py
```

### 3. Verificar a instalação

Execute o comando abaixo para verificar se o módulo Python pode ser importado:

```bash
python3 -c "from kdl_parser_py.urdf import treeFromUrdfModel; print('kdl_parser_py OK')"
```

Se a instalação estiver correta, o terminal deverá apresentar:

```text
kdl_parser_py OK
```


# Configuração do RealSense no Gazebo e edição do jaco_sim pelo VS Code

## 1. Instalação das dependências do RealSense

Inicialmente, tentei instalar os pacotes `realsense2-description` e `realsense2-gazebo`:

```bash
apt-get install -y ros-noetic-realsense2-description ros-noetic-realsense2-gazebo
```

Entretanto, ocorreu um erro durante a instalação. Como alternativa, atualizei a lista de pacotes:

```bash
apt-get update
```

Em seguida, instalei os pacotes disponíveis necessários:

```bash
apt-get install -y ros-noetic-realsense2-description ros-noetic-realsense2-camera
```

## 2. Instalação do plugin RealSense para Gazebo

Como o plugin necessário não foi instalado pelo comando anterior, foi necessário clonar o repositório com a implementação adaptada para ROS Noetic.

**2.1. Acessar o diretório de código-fonte do workspace:**

```bash
cd /root/catkin_ws/src/
```

**2.2. Clonar o repositório:**

```bash
git clone https://github.com/SgtVincent/realsense_gazebo_plugin.git
```

**2.3. Voltar para a raiz do workspace:**

```bash
cd /root/catkin_ws/
```

**2.4. Compilar o workspace:**

```bash
catkin_make
```

## 3. Carregar os ambientes ROS

Após a compilação, é necessário carregar os ambientes na seguinte ordem:

**3.1. Carregar o ambiente ROS Noetic:**

```bash
source /opt/ros/noetic/setup.bash
```

**3.2. Carregar o ambiente do workspace:**

```bash
source devel/setup.bash
```

**3.3. Iniciar a simulação do Jaco com a câmera RealSense:**

```bash
roslaunch jaco_sim jaco_realsense.launch
```

## 4. Mover a pasta jaco_sim para edição no VS Code

Para facilitar a edição dos arquivos pelo VS Code, foi necessário mover a pasta `jaco_sim` para dentro de `my_workspace`.

**4.1. No terminal do container Docker, acessar a raiz do workspace:**

```bash
cd /root/catkin_ws/
```

**4.2. Mover a pasta:**

```bash
mv /root/catkin_ws/src/jaco_sim /root/catkin_ws/src/my_workspace/
```

Com isso, a pasta `jaco_sim` passa a ficar disponível dentro do diretório `my_workspace`, utilizado para acessar e editar os arquivos pelo VS Code.

## 5. Restaurar a pasta jaco_sim para execução no Docker

Após finalizar as alterações no VS Code, é necessário devolver a pasta `jaco_sim` ao diretório `src` do workspace ROS.

**5.1. No terminal do container Docker, executar:**

```bash
mv /root/catkin_ws/src/my_workspace/jaco_sim /root/catkin_ws/src/
```

## 6. Reexecutar a simulação após alterações no VS Code

Sempre que modificar algum arquivo:

1. Salve as alterações no VS Code com `Ctrl + S`.
2. Encerre a simulação do Gazebo com `Ctrl + C` no terminal em que ela está sendo executada.
3. Execute novamente os comandos abaixo, respeitando a ordem.

**6.1. Carregar o ambiente ROS Noetic:**

```bash
source /opt/ros/noetic/setup.bash
```

**6.2. Carregar o ambiente do workspace:**

```bash
source devel/setup.bash
```

**6.3. Iniciar novamente a simulação:**

```bash
roslaunch jaco_sim jaco_realsense.launch
```

## 7. Corrigir permissões dos arquivos para edição no VS Code

Como alguns arquivos foram criados pelo container Docker utilizando o usuário `root`, pode ocorrer um problema de permissão ao tentar editá-los pelo usuário do ASUSBook.

Para corrigir isso, execute o comando abaixo no **terminal do ASUSBook (fora do Docker)**:

```bash
sudo chown -R $USER:$USER ~/projects/LaR/jaco/
```


