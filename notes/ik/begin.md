A validação da IK é feita pelo teste do laço fechado:

- Escolhemos um q_alvo (ângulos das juntas) qualquer.

- Jogamos esse q_alvo na nossa FK validada (aquela de 0,085 mm) para descobrir a pose da garra (Matriz 4x4).

- Entregamos essa Matriz 4x4 para o KDL resolver a IK e nos devolver os ângulos q_kdl.

- O teste final: Jogamos o q_kdl de volta na nossa FK. Se a pose resultante for igual à pose original, a IK do KDL funcionou perfeitamente.
    
(Nota: O q_kdl pode ser diferente do q_alvo, mas a pose da garra no espaço TEM que ser a mesma).