# Projeto Jaco — Validação Cinemática e Simulação

Este repositório contém a implementação e a validação da **cinemática direta (FK)** e da **cinemática inversa (IK)** do robô **Kinova Jaco Gen2**, tanto em ambiente simulado (**Gazebo**) quanto no **hardware real** do laboratório.

## Modelo do Robô

O modelo utilizado neste projeto é o **`j2n6s300`**, correspondente ao Jaco Gen2 com punho não esférico (*Curved Wrist*) e gripper de três dedos.

| Trecho | Significado |
|---|---|
| `j2` | Jaco, geração 2 (Gen2) |
| `s` ou `n` | Punho **esférico (`s`)** ou **não esférico (`n`)** |
| `6` | 6 graus de liberdade |
| `s` | Versão **Service** (existe também a versão **Assistive**) |
| `300` | Gripper de 3 dedos |

> **Modelo utilizado:** `j2n6s300` — Punho Não Esférico (*Curved Wrist*)

---

## Estrutura do Projeto

```text
.
├── src/
│   ├── parameters.py       # Parâmetros D1..D6, e2, aa, d4b..d6b
│   └── dh_classic.py       # Matriz DH e conversão de ângulos
│
├── tests/
│   ├── fk/
│   │   ├── fk_urdf_vs_tf.py
│   │   └── fk_dh_vs_tf.py
│   │
│   └── ik/
│       ├── ik_kdl_vs_fk.py
│       └── ik_real_test.py
│
└── jaco_sim/               # Pacote ROS: launch, URDF e câmera
```

### Descrição dos principais arquivos

- **`src/parameters.py`**  
  Contém os parâmetros geométricos utilizados nos cálculos cinemáticos, como `D1..D6`, `e2`, `aa` e `d4b..d6b`.

- **`src/dh_classic.py`**  
  Implementa a matriz de transformação baseada na convenção DH clássica e as conversões de ângulos utilizadas pelo modelo do Jaco.

- **`tests/fk/`**  
  Contém os testes de validação da cinemática direta, comparando os resultados do modelo matemático com as transformações obtidas no ROS/TF.

- **`tests/ik/`**  
  Contém os testes de cinemática inversa, incluindo a validação utilizando o KDL e os testes realizados no hardware real.

- **`jaco_sim/`**  
  Pacote ROS utilizado na simulação do Jaco no Gazebo, incluindo o modelo URDF e a integração com a câmera RealSense.

---

# Resultados

A validação foi realizada em dois ambientes:

1. **Simulação:** Gazebo com ROS Noetic.
2. **Hardware real:** Kinova Jaco Gen2 disponível no laboratório.

| Ambiente | Modelo | Validação | Resultado |
|---|---|---|---|
| Hardware real | `j2n6s300` | FK | Erro de **0,076 mm** |
| Hardware real | `j2n6s300` | FK — orientação | **0,0000°** |
| Hardware real | `j2n6s300` | IK + validação via FK | Erro de **0,076 mm** |

---

# 1. Simulação — Gazebo

A simulação foi construída em um **container Docker com ROS Noetic**.

O pacote `jaco_sim` unifica o modelo do robô e a integração com a câmera **RealSense**.

## 1.1. Compilar o Workspace

No container:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## 1.2. Rodar o Gazebo com Câmera

```bash
roslaunch jaco_sim jaco_realsense.launch
```

A câmera publica os tópicos no namespace:

```text
/camera/
```

## 1.3. Visualizar os Tópicos da Câmera

Em um segundo terminal:

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

rostopic list | grep camera
```

Para visualizar as imagens:

```bash
rqt_image_view
```

Na janela do `rqt_image_view`, selecione o tópico:

```text
/camera/color/image_raw
```

---

# 2. Validação no Robô Real

Esta seção documenta a configuração realizada no computador do laboratório (**`lar-desktop-jaco`**) para conectar o hardware e validar a cinemática do Jaco.

## 2.1. Permissão USB — Regra Udev

Para permitir que o ROS acesse o robô via USB sem a necessidade de executar os comandos como `sudo`, foi criada uma regra `udev`.

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="22cd", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-kinova.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 2.2. Configuração da Rede ROS

No computador do laboratório, o arquivo `~/.bashrc` foi configurado para utilizar o ROS Master local:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=192.168.13.100
```

Após alterar o arquivo, carregue as configurações:

```bash
source ~/.bashrc
```

---

## 2.3. Iniciar o Driver do Robô Real

Com o robô ligado e o cabo USB conectado:

```bash
source ~/jaco_ws/devel/setup.bash

roslaunch kinova_bringup kinova_robot.launch \
  kinova_robotType:=j2n6s300
```

---

## 2.4. Verificar os Tópicos

Em outro terminal:

```bash
source ~/jaco_ws/devel/setup.bash

rostopic list | grep -i joint
```

O tópico de estado utilizado pelo driver é:

```text
/j2n6s300_driver/out/joint_state
```

Esse tópico fornece os estados das juntas utilizados pelos scripts de validação.

---

# 3. Testes de Cinemática

Os scripts localizados em `tests/` foram desenvolvidos para validar os modelos de FK e IK.

## 3.1. Cinemática Direta — FK

A validação da FK compara o resultado obtido a partir da implementação da **cinemática baseada em DH** com as transformações fornecidas pelo **TF do ROS**.

No computador do laboratório:

```bash
source /opt/ros/noetic/setup.bash
source ~/jaco_ws/devel/setup.bash

python3 tests/fk/fk_dh_vs_tf.py
```

O script utiliza os estados das juntas publicados pelo driver do Jaco e compara a posição e orientação calculadas pelo modelo DH com as informações do TF.

---

## 3.2. Cinemática Inversa — IK

A validação da IK é realizada a partir do ```envio de uma trajetória ao robô``` e posterior verificação da posição alcançada utilizando a FK.

```bash
source /opt/ros/noetic/setup.bash
source ~/jaco_ws/devel/setup.bash

python3 tests/ik/ik_real_test.py
```

O resultado é validado comparando a posição desejada com a posição obtida a partir da configuração final das juntas.


# 4. Resumo do Fluxo de Validação

```text
                  Kinova Jaco Gen2
                         │
              ┌──────────┴──────────┐
              │                     │
          Simulação              Hardware
           Gazebo                Laboratório
              │                     │
              ▼                     ▼
          ROS Noetic             ROS Noetic
              │                     │
              ├──────────┬──────────┤
              │          │          │
              ▼          ▼          ▼
             FK         IK        FK / IK
              │          │          │
              └──────────┴──────────┘
                         │
                         ▼
                  Validação dos
                resultados cinemáticos
```

## Requisitos

O projeto utiliza:

- **ROS Noetic**
- **Gazebo**
- **Python 3**
- **KDL**
- **Kinova Jaco Gen2**
- **Docker** para o ambiente de simulação
- **RealSense** para a câmera da simulação
