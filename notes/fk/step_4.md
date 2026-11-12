# Mover o robô e repetir o teste

**Objetivo:** enviar um objetivo de juntas ao robô simulado, verificar que ele chegou e repetir a comparação FK vs TF numa configuração nova. Isso cobre os itens 7 e 8 dos objetivos.

## Conceitos (da sua lista, na ordem)

- **Configuração de juntas:** um vetor com 6 ângulos. É o que vamos enviar, e não uma cinemática inversa.
- **Trajetória:** uma sequência de configurações com tempo (`time_from_start`). O controlador interpola entre elas.
- **Executar:** o controlador (no seu caso, por esforço/PID) aplica torques para seguir a trajetória. Enviar não é o mesmo que chegar.
- **Verificar:** ler as juntas de volta e comparar com o objetivo. É a única prova de que houve movimento.

## Segurança antes de mover

Vamos mexer só nas juntas **1, 4 e 6**, que são `continuous` (sem limite de ângulo) e, com o braço nessa pose, giram sem risco de colisão. Isso é uma avaliação minha da geometria, e você vai observar o movimento no Gazebo. Se algo parecer errado, o robô pode ser trazido de volta com o comando de volta mais abaixo.

No Terminal B (contêiner), na ordem:

## 1. Confirmar que o controlador de trajetória está ativo

```bash
rosservice call /j2s6s300/controller_manager/list_controllers
```

- Pergunta ao gerenciador de controladores quais existem e em que estado.
- **Esperado:** `effort_joint_trajectory_controller` com `state: running`.
- **Se não estiver `running`, pare aqui** e me envie a saída.

### Resultado no terminal

```bash
controller: 
  - 
    name: "effort_joint_trajectory_controller"
    state: "running"
    type: "effort_controllers/JointTrajectoryController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_1
          - j2s6s300_joint_2
          - j2s6s300_joint_3
          - j2s6s300_joint_4
          - j2s6s300_joint_5
          - j2s6s300_joint_6
  - 
    name: "effort_finger_trajectory_controller"
    state: "running"
    type: "effort_controllers/JointTrajectoryController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_1
          - j2s6s300_joint_finger_2
          - j2s6s300_joint_finger_3
  - 
    name: "finger_tip_1_position_controller"
    state: "running"
    type: "effort_controllers/JointPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_tip_1
  - 
    name: "finger_tip_2_position_controller"
    state: "running"
    type: "effort_controllers/JointPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_tip_2
  - 
    name: "finger_tip_3_position_controller"
    state: "running"
    type: "effort_controllers/JointPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_tip_3
  - 
    name: "joint_state_controller"
    state: "running"
    type: "joint_state_controller/JointStateController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::JointStateInterface"
        resources: []
```

## 2. Enviar o objetivo

```bash
rostopic pub -1 /j2s6s300/effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory "{joint_names: ['j2s6s300_joint_1','j2s6s300_joint_2','j2s6s300_joint_3','j2s6s300_joint_4','j2s6s300_joint_5','j2s6s300_joint_6'], points: [{positions: [0.5, 2.9, 1.3, -2.09, 1.4, 0.5], time_from_start: {secs: 5}}]}"
```

### Resultado

```bash
root@asusbook:~/catkin_ws# rostopic pub -1 /j2s6s300/effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory "{joint_names: ['j2s6s300_joint_1','j2s6s300_joint_2','j2s6s300_joint_3','j2s6s300_joint_4','j2s6s300_joint_5','j2s6s300_joint_6'], points: [{positions: [0.5, 2.9, 1.3, -2.09, 1.4, 0.5], time_from_start: {secs: 5}}]}"
publishing and latching message for 3.0 seconds
root@asusbook:~/catkin_ws# rostopic info /j2s6s300/effort_joint_trajectory_controller/command
Type: trajectory_msgs/JointTrajectory

Publishers: None

Subscribers: 
 * /gazebo (http://asusbook:37815/)


root@asusbook:~/catkin_ws# rostopic list | grep effort_joint_trajectory_controller
/j2s6s300/effort_joint_trajectory_controller/command
/j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/cancel
/j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/feedback
/j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/goal
/j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/result
/j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/status
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_1/parameter_descriptions
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_1/parameter_updates
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_2/parameter_descriptions
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_2/parameter_updates
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_3/parameter_descriptions
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_3/parameter_updates
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_4/parameter_descriptions
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_4/parameter_updates
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_5/parameter_descriptions
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_5/parameter_updates
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_6/parameter_descriptions
/j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_6/parameter_updates
/j2s6s300/effort_joint_trajectory_controller/state
root@asusbook:~/catkin_ws# rostopic echo -n 1 /j2s6s300/effort_joint_trajectory_controller/state
WARNING: no messages received and simulated time is active.
Is /clock being published?
^Croot@asusbook:~/catkin_ws# rostopic echo -n 1 /j2s6s300/joint_states
WARNING: no messages received and simulated time is active.
Is /clock being published?
^Croot@asusbook:~/catkin_ws# rostopic list | grep joint_states
/j2s6s300/joint_states
root@asusbook:~/catkin_ws# rostopic echo -n 3 /clock
clock: 
  secs: 209
  nsecs: 263000000
---
clock: 
  secs: 209
  nsecs: 264000000
---
clock: 
  secs: 209
  nsecs: 265000000
---
root@asusbook:~/catkin_ws# rosservice call /gazebo/get_physics_properties
time_step: 0.001
pause: False
max_update_rate: 1000.0
gravity: 
  x: 0.0
  y: 0.0
  z: -9.8
ode_config: 
  auto_disable_bodies: False
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20
success: True
status_message: "GetPhysicsProperties: got properties"
root@asusbook:~/catkin_ws# rosservice list | grep -E 'unpause_physics|pause_physics'
/gazebo/pause_physics
/gazebo/unpause_physics
root@asusbook:~/catkin_ws# rosservice call /gazebo/unpause_physics

root@asusbook:~/catkin_ws# rostopic echo -n 1 /gazebo/model_states
name: 
  - ground_plane
  - j2s6s300
pose: 
  - 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - 
    position: 
      x: -4.637177599137456e-06
      y: -6.460962353145625e-07
      z: 9.603806646607538e-07
    orientation: 
      x: -2.5001722481780573e-06
      y: 1.9989278723953465e-05
      z: 7.319232364240158e-08
      w: 0.9999999997970864
twist: 
  - 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  - 
    linear: 
      x: -4.1280402686467814e-05
      y: -2.929532033184514e-05
      z: -0.00029895737217491115
    angular: 
      x: -0.0003340978681584682
      y: 0.0007605306325449728
      z: -3.6517965898315794e-05
---
root@asusbook:~/catkin_ws# rostopic info /j2s6s300/effort_joint_trajectory_controller/state
Type: control_msgs/JointTrajectoryControllerState

Publishers: 
 * /gazebo (http://asusbook:37815/)

Subscribers: None


root@asusbook:~/catkin_ws# rostopic list | grep j2s6s300/joint_states
/j2s6s300/joint_states
root@asusbook:~/catkin_ws# rostopic info /j2s6s300/joint_states
Type: sensor_msgs/JointState

Publishers: 
 * /gazebo (http://asusbook:37815/)

Subscribers: 
 * /j2s6s300_robot_state_publisher (http://asusbook:38253/)


root@asusbook:~/catkin_ws# rosnode info /gazebo
--------------------------------------------------------------------------------
Node [/gazebo]
Publications: 
 * /clock [rosgraph_msgs/Clock]
 * /gazebo/link_states [gazebo_msgs/LinkStates]
 * /gazebo/model_states [gazebo_msgs/ModelStates]
 * /gazebo/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /gazebo/parameter_updates [dynamic_reconfigure/Config]
 * /gazebo/performance_metrics [gazebo_msgs/PerformanceMetrics]
 * /j2s6s300/effort_finger_trajectory_controller/follow_joint_trajectory/feedback [control_msgs/FollowJointTrajectoryActionFeedback]
 * /j2s6s300/effort_finger_trajectory_controller/follow_joint_trajectory/result [control_msgs/FollowJointTrajectoryActionResult]
 * /j2s6s300/effort_finger_trajectory_controller/follow_joint_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_1/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_1/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_2/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_2/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_3/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_3/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_finger_trajectory_controller/state [control_msgs/JointTrajectoryControllerState]
 * /j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/feedback [control_msgs/FollowJointTrajectoryActionFeedback]
 * /j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/result [control_msgs/FollowJointTrajectoryActionResult]
 * /j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_1/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_1/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_2/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_2/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_3/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_3/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_4/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_4/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_5/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_5/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_6/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_6/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/effort_joint_trajectory_controller/state [control_msgs/JointTrajectoryControllerState]
 * /j2s6s300/finger_tip_1_position_controller/pid/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/finger_tip_1_position_controller/pid/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/finger_tip_1_position_controller/state [control_msgs/JointControllerState]
 * /j2s6s300/finger_tip_2_position_controller/pid/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/finger_tip_2_position_controller/pid/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/finger_tip_2_position_controller/state [control_msgs/JointControllerState]
 * /j2s6s300/finger_tip_3_position_controller/pid/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /j2s6s300/finger_tip_3_position_controller/pid/parameter_updates [dynamic_reconfigure/Config]
 * /j2s6s300/finger_tip_3_position_controller/state [control_msgs/JointControllerState]
 * /j2s6s300/joint_states [sensor_msgs/JointState]
 * /j2s6s300_joint_1_ft_sensor_topic [geometry_msgs/WrenchStamped]
 * /j2s6s300_joint_2_ft_sensor_topic [geometry_msgs/WrenchStamped]
 * /j2s6s300_joint_3_ft_sensor_topic [geometry_msgs/WrenchStamped]
 * /j2s6s300_joint_4_ft_sensor_topic [geometry_msgs/WrenchStamped]
 * /j2s6s300_joint_5_ft_sensor_topic [geometry_msgs/WrenchStamped]
 * /j2s6s300_joint_6_ft_sensor_topic [geometry_msgs/WrenchStamped]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /gazebo/set_link_state [gazebo_msgs/LinkState]
 * /gazebo/set_model_state [gazebo_msgs/ModelState]
 * /j2s6s300/effort_finger_trajectory_controller/command [trajectory_msgs/JointTrajectory]
 * /j2s6s300/effort_finger_trajectory_controller/follow_joint_trajectory/cancel [actionlib_msgs/GoalID]
 * /j2s6s300/effort_finger_trajectory_controller/follow_joint_trajectory/goal [control_msgs/FollowJointTrajectoryActionGoal]
 * /j2s6s300/effort_joint_trajectory_controller/command [trajectory_msgs/JointTrajectory]
 * /j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/cancel [actionlib_msgs/GoalID]
 * /j2s6s300/effort_joint_trajectory_controller/follow_joint_trajectory/goal [control_msgs/FollowJointTrajectoryActionGoal]
 * /j2s6s300/finger_tip_1_position_controller/command [std_msgs/Float64]
 * /j2s6s300/finger_tip_2_position_controller/command [std_msgs/Float64]
 * /j2s6s300/finger_tip_3_position_controller/command [std_msgs/Float64]

Services: 
 * /gazebo/apply_body_wrench
 * /gazebo/apply_joint_effort
 * /gazebo/clear_body_wrenches
 * /gazebo/clear_joint_forces
 * /gazebo/delete_light
 * /gazebo/delete_model
 * /gazebo/get_joint_properties
 * /gazebo/get_light_properties
 * /gazebo/get_link_properties
 * /gazebo/get_link_state
 * /gazebo/get_loggers
 * /gazebo/get_model_properties
 * /gazebo/get_model_state
 * /gazebo/get_physics_properties
 * /gazebo/get_world_properties
 * /gazebo/pause_physics
 * /gazebo/reset_simulation
 * /gazebo/reset_world
 * /gazebo/set_joint_properties
 * /gazebo/set_light_properties
 * /gazebo/set_link_properties
 * /gazebo/set_link_state
 * /gazebo/set_logger_level
 * /gazebo/set_model_configuration
 * /gazebo/set_model_state
 * /gazebo/set_parameters
 * /gazebo/set_physics_properties
 * /gazebo/spawn_sdf_model
 * /gazebo/spawn_urdf_model
 * /gazebo/unpause_physics
 * /j2s6s300/controller_manager/list_controller_types
 * /j2s6s300/controller_manager/list_controllers
 * /j2s6s300/controller_manager/load_controller
 * /j2s6s300/controller_manager/reload_controller_libraries
 * /j2s6s300/controller_manager/switch_controller
 * /j2s6s300/controller_manager/unload_controller
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_1/set_parameters
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_2/set_parameters
 * /j2s6s300/effort_finger_trajectory_controller/gains/j2s6s300_joint_finger_3/set_parameters
 * /j2s6s300/effort_finger_trajectory_controller/query_state
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_1/set_parameters
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_2/set_parameters
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_3/set_parameters
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_4/set_parameters
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_5/set_parameters
 * /j2s6s300/effort_joint_trajectory_controller/gains/j2s6s300_joint_6/set_parameters
 * /j2s6s300/effort_joint_trajectory_controller/query_state
 * /j2s6s300/finger_tip_1_position_controller/pid/set_parameters
 * /j2s6s300/finger_tip_2_position_controller/pid/set_parameters
 * /j2s6s300/finger_tip_3_position_controller/pid/set_parameters


contacting node http://asusbook:37815/ ...
Pid: 166
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (49805 - 127.0.0.1:58358) [24]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo
    * direction: outbound
    * transport: INTRAPROCESS
 * topic: /clock
    * to: /j2s6s300/j2s6s300_trajectory_controller
    * direction: outbound (49805 - 127.0.0.1:58392) [53]
    * transport: TCPROS
 * topic: /clock
    * to: /rosout
    * direction: outbound (49805 - 127.0.0.1:58444) [71]
    * transport: TCPROS
 * topic: /clock
    * to: /j2s6s300_robot_state_publisher
    * direction: outbound (49805 - 127.0.0.1:58448) [48]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo_gui
    * direction: outbound (49805 - 127.0.0.1:58454) [50]
    * transport: TCPROS
 * topic: /j2s6s300/joint_states
    * to: /j2s6s300_robot_state_publisher
    * direction: outbound (49805 - 127.0.0.1:58610) [55]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://asusbook:37815/)
    * direction: inbound
    * transport: INTRAPROCESS

root@asusbook:~/catkin_ws# 
root@asusbook:~/catkin_ws# rostopic echo -n 1 /j2s6s300/joint_states
WARNING: no messages received and simulated time is active.
Is /clock being published?
^Croot@asusbook:~/catkin_ws# rosnode info /j2s6s300/j2s6s300_trajectory_controller
--------------------------------------------------------------------------------
Node [/j2s6s300/j2s6s300_trajectory_controller]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /clock [rosgraph_msgs/Clock]

Services: 
 * /j2s6s300/j2s6s300_trajectory_controller/get_loggers
 * /j2s6s300/j2s6s300_trajectory_controller/set_logger_level


contacting node http://asusbook:42575/ ...
Pid: 105
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (44041 - 127.0.0.1:33262) [9]
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://asusbook:37815/)
    * direction: inbound
    * transport: TCPROS

root@asusbook:~/catkin_ws# rosservice call /j2s6s300/effort_joint_trajectory_controller/query_state "time: {secs: 209, nsecs: 0}"
ERROR: service [/j2s6s300/effort_joint_trajectory_controller/query_state] responded with an error: b''
root@asusbook:~/catkin_ws# 

``` 

- `rostopic pub -1 <tópico> <tipo> '<dados>'` publica **uma** mensagem e sai.
- O `joint_names` define a **ordem** dos ângulos em `positions`. Se a ordem estiver errada, o robô vai para a pose errada.
- Os valores são: junta 1 e junta 6 em 0,5 rad (≈28,6°) e as outras 4 juntas praticamente onde estão (diferenças de até ~0,01 rad).
- `time_from_start: 5` leva 5 s de **simulação**, e pode demorar mais no relógio real com renderização por software.
- **Esperado:** o braço gira a base e o punho no Gazebo. Se **nada** se mover, repita o comando uma vez (às vezes a conexão com o controlador ainda não existe quando a mensagem sai).

## 3. Esperar o movimento acabar (~15 s) e verificar a chegada

```bash
rostopic echo -n 1 /j2s6s300/joint_states/position
rostopic echo -n 1 /j2s6s300/joint_states/velocity
```

- Compare cada ângulo com o objetivo `[0.5, 2.9, 1.3, -2.09, 1.4, 0.5]`.
- Seu critério de sucesso é: erro por junta menor que **0,02 rad**, que é a tolerância `goal` configurada no controlador (nós a vimos nos parâmetros do lançamento).
- Esse número só é *checado* quando se usa a ação `follow_joint_trajectory`, e não neste atalho por tópico, então a verificação é sua.

## 4. Repetir a FK nesta nova configuração

```bash
python3 /root/catkin_ws/src/my_workspace/tests/fk_urdf_vs_tf.py 2>&1 | grep -v hardwareInterface
```

- É o mesmo script. O `2>&1 | grep -v hardwareInterface` só esconde as 12 linhas de aviso, para a saída ficar legível.

## Para voltar à pose original, se quiser (ou se algo der errado)

```bash
rostopic pub -1 /j2s6s300/effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory "{joint_names: ['j2s6s300_joint_1','j2s6s300_joint_2','j2s6s300_joint_3','j2s6s300_joint_4','j2s6s300_joint_5','j2s6s300_joint_6'], points: [{positions: [0.0, 2.9, 1.3, -2.09, 1.4, 0.0], time_from_start: {secs: 5}}]}"
```

## O que enviar e observar

1. A saída do `list_controllers` (só o trecho do `effort_joint_trajectory_controller`) e as saídas dos passos 3 e 4.
2. **Uma observação sua no Gazebo:** vendo o robô **de cima**, a base girou no sentido horário ou anti-horário com o `q1` positivo? Pelo URDF, o eixo da junta 1 aponta para −z da base, o que **sugere** sentido horário. Isto é uma previsão a testar, e não um fato.
3. Seus erros por junta (objetivo menos leitura), em uma linha.


### Resultado do terminal

```bash
anahougaz@asusbook:/$ docker exec -it kinova_dev bash
root@asusbook:~/catkin_ws# rosservice call /j2s6s300/controller_manager/list_controllers
controller: 
  - 
    name: "effort_joint_trajectory_controller"
    state: "running"
    type: "effort_controllers/JointTrajectoryController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_1
          - j2s6s300_joint_2
          - j2s6s300_joint_3
          - j2s6s300_joint_4
          - j2s6s300_joint_5
          - j2s6s300_joint_6
  - 
    name: "effort_finger_trajectory_controller"
    state: "running"
    type: "effort_controllers/JointTrajectoryController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_1
          - j2s6s300_joint_finger_2
          - j2s6s300_joint_finger_3
  - 
    name: "finger_tip_1_position_controller"
    state: "running"
    type: "effort_controllers/JointPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_tip_1
  - 
    name: "finger_tip_2_position_controller"
    state: "running"
    type: "effort_controllers/JointPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_tip_2
  - 
    name: "finger_tip_3_position_controller"
    state: "running"
    type: "effort_controllers/JointPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::EffortJointInterface"
        resources: 
          - j2s6s300_joint_finger_tip_3
  - 
    name: "joint_state_controller"
    state: "running"
    type: "joint_state_controller/JointStateController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::JointStateInterface"
        resources: []
root@asusbook:~/catkin_ws# rostopic pub -1 /j2s6s300/effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory "{joint_names: ['j2s6s300_joint_1','j2s6s300_joint_2','j2s6s300_joint_3','j2s6s300_joint_4','j2s6s300_joint_5','j2s6s300_joint_6'], points: [{positions: [0.5, 2.9, 1.3, -2.09, 1.4, 0.5], time_from_start: {secs: 5}}]}"
publishing and latching message for 3.0 seconds
root@asusbook:~/catkin_ws# rostopic echo -n 1 /j2s6s300/joint_states/position
[0.4999903582933545, 2.9006028586668497, 1.2987864215746727, -2.093335927169423, 1.4012528175470615, 0.5000002433132096, 0.9993975834233044, 1.0006109077202554, 0.9999109320363422, 0.0003425707442241688, 0.002401439361279678, 0.005962323593321273]
---
root@asusbook:~/catkin_ws# rostopic echo -n 1 /j2s6s300/joint_states/velocity
[-0.009666776998217849, -0.0007896179225540383, 0.001596947017006813, -0.004323822467522484, -0.0067995255552471354, -0.11481661061537744, -4.818476325632864, 1.3837291789185096, 0.7108715998431829, 20.121236776225743, -4.940828425279124, -3.4003065494109355]
---
root@asusbook:~/catkin_ws# python3 /root/catkin_ws/src/my_workspace/tests/fk_urdf_vs_tf.py 2>&1 | grep -v hardwareInterface
Cadeia: ['j2s6s300_joint_1', 'j2s6s300_joint_2', 'j2s6s300_joint_3', 'j2s6s300_joint_4', 'j2s6s300_joint_5', 'j2s6s300_joint_6', 'j2s6s300_joint_end_effector']
q (rad): [0.50002, 2.90059, 1.29879, -2.09335, 1.40117, 0.5]
Posição FK (m): [-0.04107  0.26797  0.53571]
Posição TF (m): [-0.04107  0.26796  0.5357 ]
Erro de posição (mm): 0.0167
Erro de orientação (graus): 0.0059
root@asusbook:~/catkin_ws# rostopic pub -1 /j2s6s300/effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory "{joint_names: ['j2s6s300_joint_1','j2s6s300_joint_2','j2s6s300_joint_3','j2s6s300_joint_4','j2s6s300_joint_5','j2s6s300_joint_6'], points: [{positions: [0.0, 2.9, 1.3, -2.09, 1.4, 0.0], time_from_start: {secs: 5}}]}"
publishing and latching message for 3.0 seconds

``` 
# O que as evidências provam

## 1. Condição prévia

O `effort_joint_trajectory_controller` estava `running` e controlando exatamente as juntas 1 a 6. Por isso, o comando pôde ser enviado com segurança.

## 2. Chegada ao objetivo

Calculei as diferenças (lido − objetivo), em rad:

| Junta | 1     | 2       | 3       | 4           | 5       | 6    |
| ----- | ----- | ------- | ------- | ----------- | ------- | ---- |
| Erro  | −1e-5 | +6,0e-4 | −1,2e-3 | **−3,3e-3** | +1,3e-3 | 2e-7 |

O maior erro, 3,3 mrad (≈0,19°) na junta 4, é aproximadamente 6 vezes menor que a tolerância de 0,02 rad.

O critério de chegada foi cumprido, e você **verificou a chegada lendo as juntas**, e não apenas observando o robô se movimentar.

## 3. FK em uma configuração nova (q1 = q6 = 0,5)

| Medida             | Valor    |
| ------------------ | -------- |
| Erro de posição    | 0,017 mm |
| Erro de orientação | 0,006°   |

Esse resultado é melhor que o da pose inicial. Agora, todas as juntas já foram exercitadas com valores não nulos.

A junta 6 gira em torno do eixo do próprio efetuador, então ela só altera a **orientação**. Os aproximadamente 28,6° dela coincidiram dentro de 0,006°, o que confirma o sinal e a ordem dessa junta.

A junta 1 moveu a posição em aproximadamente 12 cm, o que também correspondeu ao esperado.

## 4. Por que o robô não chega exatamente?

Os erros residuais sugerem um **erro de regime permanente** causado pela gravidade.

O controlador é apenas proporcional (`i = 0` nos parâmetros), com `p = 5000` nas juntas 1 a 3.

No primeiro teste, o esforço lido na junta 3 era de aproximadamente 6,05 N·m. Já o produto de `5000 × 1,2e-3 rad` resulta em aproximadamente 6,1 N·m.

Na junta 2, foram observados aproximadamente 3,12 N·m, contra `5000 × 6e-4 = 3,0 N·m`.

A coincidência é forte, mas continua sendo uma **hipótese**, pois aquela leitura era de outro modelo e outra execução.

## 5. Consequência prática para a IK

O robô simulado nunca atinge exatamente o ângulo comandado.

Por isso, a validação compara a pose **lida** com a pose desejada, utilizando uma tolerância de ordem milimétrica.

Por exemplo, um erro de 3,3 mrad na junta 4, com um braço de aproximadamente 0,3 m, corresponde a cerca de 1 mm.

**Não confunda esse erro do controlador com erro de cinemática.**

## 6. Sobre as velocidades

A junta 6 mostrou aproximadamente 0,11 rad/s novamente, mesmo com a posição parada.

Provavelmente, isso é ruído da estimativa de velocidade do Gazebo.

**Hipótese:** não use a velocidade para decidir se o robô parou. Utilize a variação da **posição**.