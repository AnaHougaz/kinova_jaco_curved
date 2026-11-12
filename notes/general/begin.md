
### O nome do modelo codifica as características do braço:

| Trecho | Significado |
|:---|:---|
| `j2` | Jaco, geração 2 (Gen2) |
| `s` ou `n` | Punho **s**férico ou **n**ão esférico |
| `6` | 6 graus de liberdade |
| `s` | Versão *Service* (existe também a versão *Assistive*) |
| `300` | Gripper de 3 dedos |

### Estamos usando o ```j2s6s300```

## O que significa validar a cinemática?

### (a) Cinemática direta (FK – Forward Kinematics)
- Dado um vetor de juntas `q`, determinar a posição e a orientação do efetuador.
- Comparar três fontes:
  1. FK implementada por DH.
  2. FK obtida pelo URDF/TF do Gazebo.
  3. FK calculada pelo KDL (via MoveIt).
- Se as três concordarem, com erros na ordem de milímetros e aproximadamente `0,01°`, a FK está validada.
- **Situação atual:** você já realizou a comparação entre (a) e (b). Ainda faltam os testes com múltiplas configurações e a comparação com o KDL.

### (b) Cinemática inversa (IK – Inverse Kinematics)
- Dada uma pose desejada do efetuador, determinar quais valores de juntas `q` permitem alcançá-la.
- A validação é feita por meio do fechamento do ciclo (loop closure):
  1. Informar uma pose desejada.
  2. Executar a IK para obter os valores de `q`.
  3. Aplicar sua FK aos valores de `q` encontrados.
  4. Comparar a pose calculada pela FK com a pose desejada.
- Se `FK(IK(pose)) ≈ pose`, a IK fecha o ciclo e apresenta consistência cinemática.
- Repetir o procedimento para:
  1. Sua implementação de IK numérica.
  2. O solucionador KDL.
- Comparar os valores de `q` encontrados pelos dois métodos e os respectivos erros de posição e orientação.
- **Objetivo:** validar a IK mesmo sem possuir uma solução analítica completa como referência.


#### O tópico /j2s6s300/joint_states tem os campos:

- name: o nome de cada junta
- position: o ângulo de cada junta, em rad
- velocity: a velocidade de cada junta, em rad/s
- effort: o torque de cada junta, em N·m

Ao rodar `rostopic echo -n 1 /j2s6s300/joint_states/velocity`, você pediu só o campo velocity de uma mensagem