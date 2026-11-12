# Testes de cinemática — Jaco Gen2 (j2s6s300) no Gazebo

Este diretório contém os testes que validam a cinemática direta (FK) do
Kinova Jaco Gen2 simulado no Gazebo. O objetivo do projeto é comparar três
fontes independentes de FK:

- **(a)** DH clássica (parâmetros da Tabela 5 do artigo
  Popov et al., *Electronics* 2024, doi 10.3390/electronics13081534)
- **(b)** URDF do pacote `kinova-ros` (usado por Gazebo, TF, MoveIt)
- **(c)** KDL do MoveIt (próximo passo)

Os testes abaixo exploram a↔b. A conclusão principal é que **(a) e (b)
descrevem o mesmo robô físico, mas com convenções de frame estruturalmente
diferentes**, e nenhuma correção simples (sinal, offset, transformação fixa)
faz as poses absolutas coincidirem. Ver "Conclusão" ao final.

## Pré-requisitos

Todos os testes rodam **dentro do container `kinova_dev`**, com Gazebo do
`j2s6s300` de pé, e ROS sourced:

    docker exec -it kinova_dev bash
    cd /root/catkin_ws/src/my_workspace
    source /opt/ros/noetic/setup.bash
    python3 tests/<nome_do_teste>.py

Para os testes que **movem o robô** (2, 7, 8, 9, 10), não mexa no Gazebo
enquanto rodam — o TF pode ficar inconsistente.

## Estrutura

    src/
      parameters.py       # D1..D6, e2  (Tabela 2 do artigo)
      dh_classic.py       # FK por DH clássica (Tabela 5) — objeto deste estudo
      transforms.py       # 6 matrizes T01..T56 (DH modificada, Tabela 6)
      fk.py               # FK por DH modificada (termina no link_6)

    tests/
      README.md               # este arquivo
      fk_urdf_vs_tf.py        # (1) valida o parser de URDF
      fk_multi_config.py      # (2) FK URDF em 4 poses
      fk_dh_vs_tf.py          # (3) DH pura × TF
      fk_dh_article_vs_tf.py  # (4) DH + eq. (50) do artigo × TF
      fk_dh_mod_vs_tf.py      # (5) DH modificada × TF (inconclusivo)
      print_tcorr_home.py     # (6) imprime T_corr em HOME
      fk_corr_diag.py         # (7) T_corr é constante entre 4 poses?
      fk_joint_isolation.py   # (8) perturba 1 junta por vez
      fk_sign_search.py       # (9) grid de sinais em q1,q4,q5
      fk_offset_search.py     # (10) otimização de offsets (δ1,δ4,δ5)

## Resumo dos resultados

| #  | Teste                    | Pergunta                                | Resultado                  |
|----|--------------------------|-----------------------------------------|----------------------------|
| 1  | fk_urdf_vs_tf.py         | Parser de URDF correto?                 | 0.047 mm / 0.010° ✅       |
| 2  | fk_multi_config.py       | FK do URDF vale em várias poses?        | < 0.1 mm / < 0.05° ✅      |
| 3  | fk_dh_vs_tf.py           | DH clássica × TF (mesma pose)?          | 39.9 mm / 105.4° ❌        |
| 4  | fk_dh_article_vs_tf.py   | Eq. (50) do artigo corrige?             | 480.6 mm / 113.0° ❌ pior  |
| 5  | fk_dh_mod_vs_tf.py       | DH modificada (Tab. 6) é comparável?    | 634 mm / 65° — inconclusivo|
| 6  | print_tcorr_home.py      | Forma de T_corr (HOME)                  | R "suja", t ≈ [85,-201,834] mm |
| 7  | fk_corr_diag.py          | T_corr é constante entre 4 poses?       | desvio 277 mm ❌           |
| 8  | fk_joint_isolation.py    | Qual junta causa divergência?           | q2,q3,q6 OK; q1,q4,q5 ≠    |
| 9  | fk_sign_search.py        | É sinal invertido?                      | não — combinação (1,1,1) ❌|
| 10 | fk_offset_search.py      | É offset δ_i?                           | não — validação 260 mm ❌  |

## Descrição dos testes

### (1) fk_urdf_vs_tf.py — validar o parser de URDF

Lê o URDF do parameter server, monta a cadeia `link_base → end_effector`
a partir de `<origin>` e `<axis>`, lê `q` de `/<model>/joint_states`, aplica
a cadeia e compara com o TF do `robot_state_publisher`.

**O que valida:** se o código Python de composição de matrizes está correto.
Não valida a geometria do robô — TF e Python leem o **mesmo URDF**, então
concordarem só prova consistência interna.

**Resultado em HOME:** 0.047 mm / 0.010° — resíduo compatível com
arredondamento e diferença de timestamp entre `joint_states` e TF.

### (2) fk_multi_config.py — FK do URDF em várias poses

Publica `JointTrajectory` para 4 configurações, lê `q` real e TF em cada
uma, compara. Salva `fk_results.csv`.

**Resultado:** < 0.1 mm / < 0.05° em todas — confirma que (1) não é
coincidência de pose.

### (3) fk_dh_vs_tf.py — DH clássica × TF (mesma pose)

Lê `q` do Gazebo, aplica `forward_kinematics_classic(q)` (Tabela 5),
compara com TF.

**O que valida:** se os parâmetros DH do artigo descrevem o mesmo robô
do URDF.

**Resultado:** 39.9 mm / 105.4°. Muito maior que (1)/(2). Não é ruído —
é diferença de convenção de frame.

### (4) fk_dh_article_vs_tf.py — DH + eq. (50) do artigo × TF

Igual a (3), mas multiplica por $T^{mobP}_{manip}$ (eq. 50 do artigo), que
corrige a transição **youBot → Jaco**.

**Resultado:** 480.6 mm / 113.0°. **Piorou** — confirma que eq. (50) é
específica do sistema móvel e **não** corrige `link_base` ↔ frame 0 da DH.

### (5) fk_dh_mod_vs_tf.py — DH modificada × TF

Compara 3 FKs com o TF na mesma pose: clássica (Tab. 5), modificada (Tab. 6,
termina em link_6) e modificada + `T6→EE` do URDF.

**Resultado:** modificada dá 634 mm, mas é **inconclusivo** — o frame 6 da
DH modificada **não é** o `link_6` do URDF; aplicar `T6→EE` em cima disso
é compor frames incompatíveis. Serve como evidência de que só a **pose
final** é comparável entre convenções diferentes.

### (6) print_tcorr_home.py — imprimir T_corr em HOME

Calcula $T_{corr} = T_{tf} \cdot T_{dh}^{-1}$ e imprime a matriz.

**Resultado:** $R_{corr}$ é "suja" (não é permutação de eixos), rotação
~105° em torno de eixo oblíquo, translação `[85, -201, 834]` mm. Sinaliza
que a diferença não é uma transformação "limpa".

### (7) fk_corr_diag.py — T_corr entre 4 poses

Move para 4 poses, calcula $T_{corr}$ em cada uma, imprime translação,
ângulo e eixo.

**Resultado:** desvio de 277 mm na translação, ângulo variando de 104° a
162°. $T_{corr}$ **não é constante** — descarta "offset de frame fixo".

### (8) fk_joint_isolation.py — perturba 1 junta por vez

Parte de HOME, perturba cada junta ±0.5 rad, mede erro em relação ao TF.

**Resultado:** q2, q3, q6 → orientação praticamente não muda (Δ < 0.1°).
q1, q4, q5 → orientação muda 1–32°. **Isola juntas suspeitas em {q1,q4,q5}.**

### (9) fk_sign_search.py — grid de sinais

Em 2 poses (HOME e PERT), testa as 8 combinações de sinal em `(q1,q4,q5)`,
escolhe a que minimiza erro.

**Resultado:** **(1,1,1)** vence nas duas. Nenhuma junta tem sinal
invertido em relação à convenção do artigo.

### (10) fk_offset_search.py — otimização de offset

Coleta 4 poses, otimiza `(δ1, δ4, δ5)` por Nelder-Mead para minimizar
a variância de $T_{corr}$, e valida numa 5ª pose que o otimizador
**não viu**.

**Resultado:** o otimizador acha `(0.135, -0.916, 1.348)` (valores
"sujos", sem interpretação geométrica), e na pose de **validação** o erro
é **260 mm** — pior que o baseline em HOME. **Hipótese descartada.**

## Conclusão do estudo

As 5 hipóteses de correção simples foram testadas e eliminadas:

| Hipótese                        | Teste | Status                |
|---------------------------------|-------|-----------------------|
| Sinal invertido em {q1,q4,q5}   | (9)   | ❌ descartada         |
| Correção eq. (50) do artigo     | (4)   | ❌ descartada (piora) |
| Offset de frame fixo            | (7)   | ❌ descartada (varia) |
| Offset por junta δ_i            | (10)  | ❌ descartada         |
| DH modificada (Tab. 6)          | (5)   | ⚠️ inconclusiva       |

**A Tabela 5 do artigo descreve o mesmo robô físico do URDF** (mesmas
dimensões, mesma estrutura de juntas), mas com **convenções de frame
estruturalmente diferentes**. Nenhuma transformação simples entre as duas
faz as poses absolutas coincidirem.

Observação lateral: o próprio artigo tem uma inconsistência interna entre
Tabela 5 (DH clássica, `d3 = -e2 = -0.0133`) e Tabela 6 (DH modificada,
`d3 = +e2 = +0.0133`). As duas tabelas deveriam descrever o mesmo robô;
uma das duas está errada.

A comparação válida entre os dois modelos é **pose-final × pose-final com
$T_{corr}$ medido em cada pose**, não em coordenadas absolutas. Isso é
característico de comparar uma DH publicada em artigo com um URDF de
fabricante: cada um define frames por razões próprias (legibilidade
matemática no artigo, compatibilidade com drivers/simuladores no URDF).

## Próximos passos (fora deste diretório)

- **Caminho A**: derivar a tabela DH **do URDF** e comparar parâmetro-por-
  parâmetro com a Tabela 5 do artigo. Identifica exatamente onde diferem.
- **Caminho B**: usar KDL do MoveIt como terceira fonte, comparar as três
  FKs e fechar a validação por outro caminho (autorizado pelo orientador).


## Conclusão do estudo (RESOLVIDO)

A divergência inicial (40 mm / 105°) foi **totalmente resolvida**. 

A Tabela 5 do artigo (DH clássica) está correta, mas o artigo **omitiu duas etapas** 
do manual oficial da Kinova (User Guide Gen2) necessárias para comparar com o URDF:

1. **Tabela 30 (Página 54 do manual):** Offsets de mapeamento dos ângulos físicos 
   (ROS) para os ângulos do algoritmo DH:
   - q1_dh = q1_fisico - 180°
   - q2_dh = q2_fisico + 90°
   - q3_dh = q3_fisico + 90°
   - q6_dh = q6_fisico - 90°
   (Além disso, o valor de `e2` no artigo está 0.0133, mas o manual do 6DoF Spherical diz 0.0098).

2. **Página 56 do manual:** Transformação da base (`link_base` do URDF) para o 
   frame 0 do algoritmo DH. A origem do DH está 551.02 mm acima da base e rotacionada 
   180° no eixo X.

Com essas duas correções implementadas em `src/dh_classic.py`, a validação final 
(`tests/fk_dh_full_vs_tf.py`) atingiu:
- **Erro de posição:** 0.085 mm (85 micrômetros)
- **Erro de orientação:** 0.010°

Isso atinge o mesmo nível de ruído do teste URDF vs TF (0.047 mm), provando que a 
matemática está exata. A evidência L2 está fechada com sucesso.