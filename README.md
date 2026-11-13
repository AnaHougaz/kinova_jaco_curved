# Projeto Jaco — Validação no Hardware Real (`hardware-validation`)

> 🌿 **Você está na branch `hardware-validation`.** 
> Esta branch contém os scripts Python adaptados especificamente para ler os dados e enviar comandos ao **robô físico** do laboratório (`j2n6s300`).
> 
> 🔗 Para a simulação no Gazebo, acesse a branch [**develop**](https://github.com/AnaHougaz/kinova_jaco_curved/tree/develop).

Este repositório contém a implementação e a validação da **cinemática direta (FK)** e **inversa (IK)** do robô **Kinova Jaco Gen2** com punho curvo (`j2n6s300`).

## Resultados Obtidos

| Ambiente | Modelo | Validação | Erro de Posição | Erro de Orientação | Link |
|---|---|---|---|---|---|
| **Hardware Real** | `j2n6s300` | FK | **0,076 mm** | **0,0000°** | [Código](tests/fk/fk_dh_vs_tf.py) |
| **Hardware Real** | `j2n6s300` | IK (Laço Fechado) | **0,076 mm** | — | [Código](tests/ik/ik_real_test.py) |
| [Simulação](https://github.com/AnaHougaz/kinova_jaco_curved/tree/develop) | `j2n6s300` | FK / IK | 0,070 mm / 0,067 mm | 0,0009° | [Branch develop](https://github.com/AnaHougaz/kinova_jaco_curved/tree/develop) |
| [Simulação](https://github.com/AnaHougaz/kinova_jaco) | `j2s6s300` | FK / IK | 0,085 mm / 0,070 mm | 0,010° | [Repo antigo](https://github.com/AnaHougaz/kinova_jaco) |

---

## 🖥️ Rotina de Execução no Laboratório (Hardware Real)

**Atenção:** No laboratório, o ROS roda **nativamente** (sem Docker) no Ubuntu 20.04. O modelo físico é o `j2n6s300`.

### 1. Preparar o Robô (Primeira vez)
Ligue o robô e conecte o cabo USB. Para dar permissão de acesso ao ROS sem precisar de `sudo`:
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="22cd", MODE="0666"' | sudo tee /etc/udev/rules.d/99-kinova.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. Configurar a Rede do ROS
O computador do laboratório deve reconhecer o próprio ROS como mestre.
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=192.168.13.100
```

### 3. Iniciar o Driver do Robô (Terminal 1)
```bash
source ~/jaco_ws/devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300
```
*(Deixe este terminal aberto. O robô dará um sinal de vida e os dedos podem se mover).*

### 4. Verificar o Tópico (Terminal 2)
Abra outro terminal e confirme que o tópico das juntas está ativo.
```bash
source ~/jaco_ws/devel/setup.bash
rostopic list | grep -i joint
```
O tópico correto é: `/j2n6s300_driver/out/joint_state`.

### 5. Rodar os Testes (Terminal 3)
Navegue até esta pasta do repositório e rode os scripts:

**Validação da FK (Cinemática Direta):**
```bash
source /opt/ros/noetic/setup.bash
source ~/jaco_ws/devel/setup.bash
python3 tests/fk/fk_dh_vs_tf.py
```

**Validação da IK (Cinemática Inversa):**
```bash
python3 tests/ik/ik_real_test.py
```
⚠️ **Atenção:** O `ik_real_test.py` move o robô físico. Tenha certeza que o professor ou técnico está com o botão de emergência em mãos.

---

## 💻 Como rodar a Simulação (Notebook / Docker)

Caso você queira reproduzir os testes da branch `develop` no seu computador pessoal, os comandos são:

```bash
# No host (fora do Docker)
xhost +local:docker
docker start -ai kinova_hardware

# Dentro do container
cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch jaco_sim jaco_realsense.launch
```

---

## Estrutura do Código

```text
.
├── src/
│   ├── parameters.py       # Parâmetros geométricos (D1..D6, aa, d4b..d6b)
│   └── dh_classic.py       # Implementação da matriz DH (sem T_base_to_0)
│
├── tests/
│   ├── fk/
│   │   ├── fk_urdf_vs_tf.py  # Validação via URDF
│   │   └── fk_dh_vs_tf.py    # Validação via DH (principal)
│   │
│   └── ik/
│       ├── ik_kdl_vs_fk.py   # Validação da IK no Gazebo
│       └── ik_real_test.py   # Validação da IK no hardware
│
└── jaco_sim/               # Pacote ROS (usado apenas na branch develop)
```

### Principais diferenças desta branch
*   **`src/dh_classic.py`:** A função `forward_kinematics_full` retorna a matriz clássica **sem** a matriz de correção `T_base_to_0` (pois para o punho curvo, a base do DH já coincide com o `link_base` do ROS).
*   **Leitura de tópicos:** Os scripts leem o tópico `/j2n6s300_driver/out/joint_state` (driver do hardware) e não `/j2n6s300/joint_states` (simulação).

---

## Solução de Problemas Comuns no Lab

*   **Erro `KinovaCommException` (1015):** O robô está desligado, o cabo USB está solto ou a regra `udev` não foi aplicada. Verifique com `lsusb` se o dispositivo Kinova aparece.
*   **Erro `timeout exceeded`:** O script está procurando o tópico no lugar errado. Confirme se o driver (`roslaunch kinova_bringup...`) está rodando no outro terminal.
*   **Erro `Permission denied` (Linux):** O script tenta acessar o hardware sem permissão. Reaplique a regra `udev` e reconecte o cabo USB.