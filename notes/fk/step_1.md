# Ler as juntas e a pose do efetuador (baseline da FK)

Objetivo: com o robô parado, registrar duas informações: os ângulos das 6 juntas e a pose do efetuador final segundo o ROS. Esse par é a referência dos testes de cinemática direta.

Conceitos:

- Cinemática direta (FK): dados os ângulos das juntas, calcula a posição e orientação da ponta do braço.
- TF: sistema do ROS que guarda a posição de cada referencial. O robot_state_publisher calcula a FK a partir do URDF e do joint_states e publica o resultado no TF. Então o TF é uma referência independente do seu código Python, mas não independente do URDF. Isso é o que queremos: validar contra o modelo usado na simulação.
- A pose só significa algo relativa a um referencial. Por isso o comando abaixo pede dois: a origem e o efetuador.

# 1. Primeiro, qual problema queremos resolver?

Imagine que você tem um braço robótico Kinova Jaco com seis juntas.

Cada junta possui um ângulo, que determina parte da configuração do braço. Por exemplo:

$$
\mathbf{q} =
\begin{bmatrix}
q_1\\
q_2\\
q_3\\
q_4\\
q_5\\
q_6
\end{bmatrix}
$$

Em que:
- $q_1$: ângulo da junta 1;
- $q_2$: ângulo da junta 2;
- E assim por diante, até $q_6$.

Esses seis ângulos determinam a configuração do braço robótico.

Agora, suponha que você queira responder à seguinte pergunta:

**Se as seis juntas estão nesses ângulos, onde está a ponta do braço robótico e para onde ela está orientada?**

Essa é justamente a pergunta respondida pela cinemática direta, ou *Forward Kinematics* (FK).

A FK recebe os ângulos das juntas e calcula a pose do efetuador final.

Matematicamente:

$$
\boxed{\mathbf{q} \longrightarrow \text{FK} \longrightarrow \mathbf{T}}
$$

Em que:
- $\mathbf{q}$ representa os ângulos das juntas;
- $\mathbf{T}$ representa a pose do efetuador final.

A pose contém duas informações:

**Posição:**
$$
\mathbf{p} =
\begin{bmatrix}
x\\y\\z
\end{bmatrix}
$$

**Orientação:** descreve como o efetuador está rotacionado em relação a um referencial.

Portanto, a pose completa reúne seis graus de liberdade espaciais: três associados à posição e três à orientação. Isso não significa que o robô tenha seis juntas adicionais: são duas maneiras de descrever a configuração espacial da sua ponta.

---

# 2. Por que precisamos ler as juntas e a pose do efetuador?

Porque queremos testar a FK que você vai implementar ou analisar no projeto.

Imagine que você escreva um código Python que recebe os seis ângulos e calcula a posição da ponta do Jaco.

Como saber se o resultado está correto?

Você precisa de algum valor de referência para comparar com o resultado do seu código.

É aí que entra a Etapa 5.

Ela consiste em registrar duas informações enquanto o robô está parado:

1. **Os seis ângulos das juntas:** a configuração atual do braço.
2. **A pose do efetuador final publicada pelo ROS:** a posição e a orientação da ponta nessa mesma configuração.

Por exemplo, suponha que você registre:

| Informação | Exemplo ilustrativo |
|---|---|
| Junta 1 | $0{,}0$ rad |
| Junta 2 | $0{,}5$ rad |
| Junta 3 | $-0{,}3$ rad |
| Junta 4 | $0{,}2$ rad |
| Junta 5 | $0{,}1$ rad |
| Junta 6 | $0{,}0$ rad |
| Posição X do efetuador | $0{,}41$ m |
| Posição Y do efetuador | $-0{,}0133$ m |
| Posição Z do efetuador | $0{,}3111$ m |

**Atenção:** esses valores são apenas ilustrativos para demonstrar a ideia. A posição de referência precisa ser medida no seu ambiente, e os ângulos precisam ser lidos do estado real das juntas na simulação.

Agora você tem um par de informações:

$$
\boxed{
\left(
\underbrace{\mathbf{q}}_{\text{ângulos observados}},
\underbrace{\mathbf{T}_{\text{ROS}}}_{\text{pose de referência}}
\right)
}
$$

Esse par será utilizado para verificar se o seu código calcula a pose correta a partir dos ângulos observados.

## Qual é a lógica do teste?

Depois de registrar os dados, você executará sua própria FK:

$$
\mathbf{T}_{\text{Python}} = FK_{\text{Python}}(\mathbf{q})
$$

Em seguida, comparará o resultado com a pose obtida pelo ROS:

$$
\mathbf{T}_{\text{Python}}
\stackrel{?}{\approx}
\mathbf{T}_{\text{ROS}}
$$

Se os resultados forem próximos, isso é uma evidência de que sua implementação está coerente com o modelo cinemático utilizado na simulação.

Se forem diferentes, será necessário investigar a causa.

Por exemplo:
- Uma junta pode estar sendo interpretada com o sinal invertido.
- Os parâmetros geométricos do robô podem estar incorretos.
- O código pode estar utilizando uma ordem de transformações errada.
- Os referenciais usados na comparação podem ser diferentes.
- A pose pode estar sendo calculada para um frame diferente do efetuador desejado.

Observe que essa comparação não serve apenas para dizer se o código está certo ou errado. Ela ajuda a localizar possíveis inconsistências.

---

# 3. Mas de onde o ROS tira a pose do efetuador?

Esse é o conceito mais importante dessa etapa.

O ROS não precisa conhecer o seu código Python de FK para calcular a pose de referência.

No ambiente que você descreveu, existe uma cadeia de componentes responsáveis por isso.

Vamos entender cada um.

### 3.1. URDF: a descrição do robô

O URDF (*Unified Robot Description Format*) descreve a estrutura do robô.

Ele informa, entre outras coisas:
- Quais são os links, ou partes rígidas, do braço.
- Quais são as juntas que conectam esses links.
- Como as juntas se movimentam.
- Os eixos de rotação ou translação.
- As relações geométricas entre os referenciais.
- Os parâmetros necessários para descrever a estrutura cinemática.

Imagine que o URDF seja a descrição matemática e estrutural do braço.

Ele diz ao ROS como as partes do robô estão conectadas e como cada junta transforma um referencial no seguinte.

Entretanto, para determinar a pose atual do efetuador, o ROS também precisa saber os ângulos atuais das juntas.

É aí que entra o próximo componente.

### 3.2. `joint_states`: os valores das juntas

O tópico `joint_states`, normalmente associado à mensagem `sensor_msgs/msg/JointState` no ROS 2, fornece o estado das juntas.

Entre as informações possíveis, estão:
- Posição das juntas.
- Velocidade.
- Esforço.

Para a FK, o que nos interessa principalmente são as posições angulares.

Assim, o ROS pode receber um vetor de posições:

$$
\mathbf{q} =
[q_1,q_2,q_3,q_4,q_5,q_6]
$$

Esses valores descrevem a configuração atual do braço.

**Um detalhe importante:** o vetor não deve ser interpretado simplesmente pela ordem em que os valores aparecem na mensagem. É necessário associar cada posição ao nome da junta correspondente, porque a ordem pode variar.

### 3.3. `robot_state_publisher`: o cálculo das transformações

O `robot_state_publisher` utiliza a descrição do robô e o estado das juntas para calcular e publicar as transformações entre os referenciais dos links. 

O `robot_state_publisher` faz o cálculo da Cinemática Direta (Forward Kinematics - FK).
A principal função deste nó no ROS (Robot Operating System) é ler os ângulos e posições atuais das juntas do robô (geralmente vindos do tópico /joint_states) e, utilizando a descrição estrutural do robô (arquivo URDF), calcular a posição e orientação de cada link (elo) no espaço 3D.
O resultado desse cálculo de Cinemática Direta é publicado continuamente no sistema de transformações do ROS, o TF (/tf e /tf_static), permitindo que todo o ecossistema saiba exatamente onde cada parte do robô está localizada em tempo real.

O processo pode ser representado assim:

```text
                URDF
                  |
                  | Estrutura e geometria
                  v
          robot_state_publisher
                  ^
                  |
                  | Ângulos das juntas
                  |
              joint_states
                  |
                  v
       Cálculo das transformações
                  |
                  v
              Árvore TF
                  |
                  v
       Pose do efetuador final
```

A representação acima é conceitual: o `robot_state_publisher` recebe a descrição do robô e as mensagens de estado das juntas, calcula as transformações e publica as transformações correspondentes no TF.

O TF é o sistema do ROS que permite consultar as relações entre referenciais.

Por exemplo, você pode consultar a transformação entre o referencial da base do robô e o referencial da ponta do braço.

Essa transformação contém a posição e a orientação de um referencial em relação ao outro.

Assim, você consegue obter a pose do efetuador sem implementar novamente a FK no seu código.

---

# 4. Por que o TF é uma boa referência para validar sua FK?

Aqui precisamos fazer uma distinção importante.

O TF não é uma medição física independente da geometria do robô. Ele contém transformações calculadas a partir do modelo e dos estados das juntas.

No seu caso, a cadeia de cálculo é:

$$
\underbrace{\text{URDF}+\texttt{joint\_states}}_{\text{entradas do ROS}}
\longrightarrow
\underbrace{\texttt{robot\_state\_publisher}}_{\text{FK do ROS}}
\longrightarrow
\underbrace{\text{TF}}_{\text{transformações publicadas}}
$$

Seu código Python fará um cálculo próprio:

$$
\underbrace{\text{URDF ou parâmetros cinemáticos}}_{\text{modelo}}
+
\underbrace{\mathbf{q}}_{\text{ângulos}}
\longrightarrow
\underbrace{FK_{\text{Python}}}_{\text{implementação própria}}
\longrightarrow
\underbrace{\mathbf{T}_{\text{Python}}}_{\text{pose calculada}}
$$

A ideia é comparar as duas saídas.

| Aspecto | FK do ROS via TF | Sua FK em Python |
|---|---|---|
| Entradas | URDF e estados das juntas | Ângulos e parâmetros do modelo |
| Método | Transformações calculadas pelo `robot_state_publisher` | Algoritmo que você implementar |
| Resultado | Pose de referência no TF | Pose calculada pelo seu código |
| Finalidade | Referência para a comparação | Implementação a ser validada |

O principal benefício é que você compara duas implementações do cálculo cinemático, em vez de comparar seu código apenas com valores que você mesmo calculou.

Entretanto, existe uma limitação fundamental:

**As duas implementações podem concordar e ainda estar erradas em relação ao robô físico**, se compartilharem um URDF incorreto ou se utilizarem a mesma convenção equivocada.

Por isso, essa etapa valida principalmente a consistência da sua implementação com o modelo cinemático da simulação.

Ela não comprova, isoladamente, que a cinemática do modelo corresponde perfeitamente ao robô físico.

---

# 5. Por que precisamos informar dois referenciais?

Agora chegamos à última parte do texto original:

> A pose só significa algo relativa a um referencial. Por isso o comando abaixo pede dois: a origem e o efetuador.

Esse conceito é essencial para trabalhar com robótica.

Imagine que a ponta do braço esteja em uma posição específica.

Você poderia descrevê-la em relação à base do robô, ao mundo da simulação ou a outro referencial.

Os números podem mudar, mesmo que o braço esteja exatamente na mesma configuração física.

Isso acontece porque **posição e orientação só podem ser interpretadas quando sabemos em relação a qual referencial elas foram expressas**.

## 5.1. O que é um referencial?

Um referencial é um sistema de coordenadas que permite descrever posições e orientações.

Em três dimensões, ele possui três eixos:

- $X$;
- $Y$;
- $Z$.

No ROS, cada link pode ter seu próprio referencial.

No Kinova Jaco, podemos ter, por exemplo:

- `base_link`: referencial associado à base do braço.
- Referenciais intermediários: associados aos links e às juntas.
- Um referencial associado ao efetuador final.

Os nomes exatos dependem do URDF utilizado no seu projeto.

## 5.2. O que significa consultar a pose da ponta em relação à base?

Suponha que você consulte a transformação entre:

```text
Referencial de origem: base_link
Referencial de destino: referencial do efetuador
```

Você está perguntando:

**Qual é a posição e a orientação do efetuador final quando descritas no sistema de coordenadas da base do robô?**

Matematicamente, podemos representar essa transformação como:

$$
{}^{B}\mathbf{T}_{E}
$$

Em que:
- $B$ representa o referencial da base;
- $E$ representa o referencial do efetuador;
- ${}^{B}\mathbf{T}_{E}$ descreve a pose do efetuador em relação à base.

A transformação homogênea tem a seguinte estrutura:

$$
{}^{B}\mathbf{T}_{E}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} & x\\
r_{21} & r_{22} & r_{23} & y\\
r_{31} & r_{32} & r_{33} & z\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Os elementos da matriz representam:

- A matriz $3\times3$ do canto superior esquerdo: orientação relativa do efetuador.
- A última coluna, nas três primeiras linhas: posição relativa do efetuador.
- A última linha: estrutura matemática da transformação homogênea.

Portanto, uma única transformação reúne a posição e a orientação que você precisa para validar a FK.

### E se você usar o referencial `world` em vez de `base_link`?

A pose será expressa em relação ao mundo da simulação.

Se a base do robô estiver posicionada ou rotacionada em relação ao mundo, os valores poderão ser diferentes.

Isso não significa necessariamente que algum cálculo esteja errado. Você simplesmente estará descrevendo a pose a partir de outro referencial.

**Para validar a FK do braço, o importante é usar o mesmo referencial de origem nas duas implementações.**

---

# 6. Por que o robô precisa estar parado?

O objetivo é estabelecer uma referência estável.

Imagine que você leia os ângulos das juntas em um instante e a pose do efetuador em outro.

Se o braço estiver se movimentando, os ângulos podem mudar entre as duas leituras.

Nesse caso, você pode acabar comparando:

$$
\mathbf{q}(t_1)
\quad\text{com}\quad
\mathbf{T}(t_2)
$$

em que $t_1$ e $t_2$ representam instantes diferentes.

Isso introduziria uma inconsistência no teste.

Com o robô parado, reduzimos esse problema porque esperamos que a configuração permaneça estável durante as leituras.

Ainda assim, é importante verificar se os dados correspondem à mesma configuração. Em uma validação mais rigorosa, você pode registrar os timestamps e conferir se os estados das juntas e a transformação consultada são compatíveis temporalmente.

Além disso, o TF pode conter transformações estáticas e dinâmicas. A transformação entre a base e o efetuador normalmente depende das juntas e, portanto, da configuração do braço.

---

# 7. Afinal, o que essa etapa entrega para o projeto?

A Etapa 5 não tem como objetivo principal implementar a FK. Ela estabelece os dados de referência necessários para testar essa implementação posteriormente.

O fluxo de trabalho é:

**1. Registrar os ângulos das juntas**

Você obtém os valores atuais das seis juntas, associando corretamente cada valor ao nome da junta.

Resultado:

$$
\mathbf{q}_{\text{referência}}
$$

**2. Consultar a pose do efetuador via TF**

Você define o referencial de origem e o referencial do efetuador.

Resultado:

$$
\mathbf{T}_{\text{ROS}}
$$

**3. Guardar os dois resultados**

Você registra os ângulos e a pose obtida para a mesma configuração.

Resultado:

$$
\left(
\mathbf{q}_{\text{referência}},
\mathbf{T}_{\text{ROS}}
\right)
$$

**4. Utilizar os dados para validar sua implementação**

Seu código recebe os mesmos ângulos e calcula a pose.

Resultado:

$$
\mathbf{T}_{\text{Python}}
$$

**5. Comparar as poses**

Você verifica se a posição e a orientação calculadas pelo seu código são compatíveis com a referência do ROS.

Para a posição, pode calcular o erro euclidiano:

$$
e_p =
\sqrt{
(x_P-x_R)^2+
(y_P-y_R)^2+
(z_P-z_R)^2
}
$$

Em que:
- $x_P,y_P,z_P$ são as coordenadas calculadas pelo Python;
- $x_R,y_R,z_R$ são as coordenadas obtidas do ROS.

Para a orientação, é necessário utilizar uma métrica apropriada para rotações, em vez de simplesmente subtrair os ângulos de Euler sem considerar suas convenções.

---

## 8. A ideia central que você deve guardar

Bia, pense nessa etapa como a construção de um **gabarito para corrigir sua implementação de cinemática direta**.

Você tem um robô com seis juntas. Primeiro, registra os ângulos que descrevem a configuração do braço. Depois, consulta no TF a pose do efetuador final para essa configuração.

O ROS calcula essa pose com base no URDF e nos estados das juntas. Seu código Python deverá calcular a pose a partir dos mesmos ângulos.

Você compara os resultados para verificar se sua implementação reproduz o comportamento cinemático esperado pelo modelo.

Em resumo:

$$
\boxed{
\text{Mesmos ângulos}
\longrightarrow
\begin{cases}
\text{FK do ROS via TF}\\
\text{FK implementada em Python}
\end{cases}
\longrightarrow
\text{Comparação das poses}
}
$$

**O racional da Etapa 5 é, portanto, estabelecer uma referência antes de testar o seu próprio cálculo.** Sem registrar os ângulos e a pose correspondente, você não teria uma base consistente para avaliar os resultados da sua implementação.


---
# CÓDIGOS

## 1. Objetivo

Registrar os ângulos das juntas e a pose do efetuador final do Kinova Jaco 6 DOF no ROS, para estabelecer uma referência de comparação com a implementação própria da cinemática direta (FK), baseada na matriz de Denavit–Hartenberg (DH) do manual do robô.


## 2. Acessar o container Docker

```bash
docker exec -it kinova_dev bash
```

O terminal passa a executar comandos dentro do container:

```bash
root@asusbook:~/catkin_ws#
```

## 3. Identificar o tópico de estado das juntas

Inicialmente, foi tentado acessar o tópico:

```bash
rostopic echo -n 1 /j2n6s300/joint_states/position
```

O ROS retornou:

```text
WARNING: topic [/j2n6s300/joint_states/position] does not appear to be published yet
```

A tentativa foi interrompida com `Ctrl+C`.

Para identificar o tópico correto, foi executado:

```bash
rostopic list | grep joint_states
```

Resultado:

```text
/j2s6s300/joint_states
```

**Conclusão:** o tópico disponível no ambiente é `/j2s6s300/joint_states`, e não `/j2n6s300/joint_states`.

Essa diferença no nome do modelo explica por que a primeira tentativa não funcionou.

## 4. Registrar as posições das juntas

Com o tópico identificado, foi executado:

```bash
rostopic echo -n 1 /j2s6s300/joint_states/position
```

Resultado:

```text
[5.63467515837246e-05, 2.900591816189075, 1.2987778260332536, -2.086533910021131, 1.4011457115457802, 8.283462449742984e-05, 0.9971967909820405, 1.0002678123868867, 1.001127790199904, -0.0002529639850861187, 0.005880217063212179, 0.003302549617134609]
---
```

Juntas (rad): [5.6e-05, 2.9006, 1.2988, -2.0865, 1.4011, 8.3e-05] para as juntas 1 a 6. As seis seguintes são os dedos, que não nos interessam agora.

Comando recomendado:

```bash
rostopic echo -n 1 /j2s6s300/joint_states
```

A mensagem completa permite verificar os campos `name`, `position`, `velocity` e `effort`.

Não se deve assumir que os seis primeiros valores correspondem às seis juntas do braço sem conferir os nomes. Os valores adicionais podem pertencer a outros elementos articulados, dependendo da configuração publicada.

## 5. Registrar as velocidades das juntas

Também foi executado:

```bash
rostopic echo -n 1 /j2s6s300/joint_states/velocity
```

Resultado:

```text
[-0.002019989602266242, -0.0005914145064306337, 0.0006851495031261679, -9.609917228040705e-05, 0.0019727247593000026, 0.1139988272210565, -0.9954721093830994, -1.5248220077775683, -0.9440805774345147, 0.573436703237387, 6.577943627516622, 3.173384882998469]
---
```

## 6. Consultar a pose do efetuador pelo TF

Foi executado o seguinte comando:

```bash
timeout 4 rosrun tf tf_echo j2s6s300_link_base j2s6s300_end_effector
```

O comando consulta a transformação entre:

- **Referencial de origem:** `j2s6s300_link_base`
- **Referencial de destino:** `j2s6s300_end_effector`

O `timeout 4` limita a execução a aproximadamente quatro segundos.

### 6.1. Resultado obtido

Uma das leituras retornadas foi:

```text
At time 267.901
- Translation: [-0.165, 0.216, 0.537]
- Rotation: in Quaternion [-0.583, 0.633, 0.294, 0.416]
            in RPY (radian) [-2.909, 1.054, -1.516]
            in RPY (degree) [-166.679, 60.403, -86.867]
```

As leituras seguintes apresentaram valores muito próximos:

```text
Translation: [-0.165, 0.216, 0.537]
Rotation in RPY (degree): aproximadamente [-166.68, 60.40, -86.88]
```

### 6.2. Interpretação

A translação representa a posição do efetuador em relação à base do robô:

\[
\mathbf{p}_{TF} =
\begin{bmatrix}
-0.165\\
0.216\\
0.537
\end{bmatrix}
\text{ m}
\]

A orientação foi publicada em duas representações:

- **Quaternion:** quatro componentes que representam a orientação espacial.
- **RPY (Roll, Pitch, Yaw):** três ângulos que descrevem a orientação segundo a convenção Roll-Pitch-Yaw utilizada pelo ROS.

Para comparar a FK implementada em Python com o TF, devem ser utilizadas representações e convenções compatíveis.

A posição está expressa em metros, enquanto os ângulos RPY apresentados estão em radianos ou graus, conforme indicado na saída.

### 6.3. Registro da referência do TF

| Grandeza | Valor |
|---|---|
| Referencial de origem | `j2s6s300_link_base` |
| Referencial de destino | `j2s6s300_end_effector` |
| Posição X | -0,165 m |
| Posição Y | 0,216 m |
| Posição Z | 0,537 m |
| Roll | -2,909 rad |
| Pitch | 1,054 rad |
| Yaw | -1,516 rad |

Esses valores constituem uma referência inicial da pose publicada pelo ROS.

## 7. Localizar a configuração MoveIt do robô

Para localizar o pacote de configuração do MoveIt, foi executado:

```bash
rospack find j2s6s300_moveit_config
```

Resultado:

```text
/root/catkin_ws/src/kinova-ros/kinova_moveit/robot_configs/j2s6s300_moveit_config
```

O pacote j2s6s300_moveit_config existe, em kinova-ros/kinova_moveit/robot_configs/.

Um alerta sobre convenções: o RPY do `tf_echo` e os “Euler XYZ” que o README da Kinova cita para o `pose_action_client` podem não ser a mesma convenção. Isso é hipótese e vamos testar. É por isso que compararemos orientações por quaternion ou matriz de rotação, e não por três ângulos.

O que esta etapa ainda não prova: uma única configuração não valida cinemática direta. Para isso precisaremos de várias configurações de juntas.

