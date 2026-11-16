# Plano da Atividade M3: estimativa de WCRT com medições em ESP32 física

> Disciplina: Sistemas em Tempo Real (UNIVALI, Eng. de Computação)
> Professores: Michael Douglas C A e Felipe Viel
> Valor: 2,0 pontos, integrados à atividade M3. Grupos de até 2 alunos.
> Entrega: AVA 2.0, até 11:35 do dia 12/11 (**conferir**: a data de hoje é 16/11/2026 e o enunciado indica 12/11).

**Decisão de escopo:** todas as medições oficiais são feitas na **ESP32 física** (ESP-IDF + FreeRTOS). Não há medição em Linux. O PC é usado apenas para compilar, rodar a análise dos CSV e medir cobertura de código (gcov), que não interfere nas medições da placa.

---

## 0. Estado atual (ler primeiro; atualizar a cada sessão)

**Última atualização:** 16/11/2026, durante a sessão de implementação da etapa 1.

### 0.1 Arquivos existentes
- `atividade_m3/main/huffman.h`: Huffman completo, **header-only** (todas as funções `static`, guarda `HUFFMAN_H`). Seções 0 a 10 na ordem do algoritmo: nó, frequências, menor, árvore, liberar, códigos, codificar, decodificar, validar, gerador xorshift32. Sem `printf` na lógica. Verificado no PC com ASan/UBSan. Revisado para ESP32 por leitura, sem mudanças necessárias (ver 0.4).
- `atividade_m3/main/experimento.c`: contém
  - `executar_experimento(Experimento *e, n, tam_texto, semente_base)`: laço de N ativações. Gera a entrada fora da medição. Mede cada etapa (criar árvore, gerar códigos, codificar, decodificar) e o total com `clock_gettime(CLOCK_MONOTONIC)`. Valida com `strcmp` e conta erros. Não imprime.
  - `static Experimento exp` (global): `tempos_us[10000]` (40 KB) e `etapas[10000]` (160 KB). Total ~200 KB.
  - `rodar_experimento()`: executa com n = 1000, tamanho = 1000, semente = 12345. Imprime HWM parcial a cada 100 ativações e o resumo (mín, média, HWM por total e por etapa).
  - `#ifdef ESP_PLATFORM`: `app_main()` cria a tarefa `experimento` (pilha 8192, prioridade 5) fixada no **core 1**. Senão: `main()` para o PC.
- `atividade_m3/main/main_teste.c`: teste de validação da Huffman (casos "paralelepipedo", "abracadabra", "aaaa", "a" e aleatório N=1000). Compila no PC.
- `atividade_m3/CMakeLists.txt` (raiz, ESP-IDF): existe.
- `atividade_m3/main/CMakeLists.txt`: **não existe** (foi apagado). O usuário vai criar pelo ESP-IDF. Não criar sem pedido.
- `atividade_m3/codificacao_huffman.c`: removido (substituído por `huffman.h`).
- Não existem ainda: exportação CSV, análise (`analysis/`), `host/` (gcov), etapas 2 a 4, `Kconfig.projbuild`, relatório, README.

### 0.2 Resultados já obtidos (só no PC, Linux, N=1000, tamanho 1000, semente 12345)
- Erros: 0 em todas as rodadas.
- Execuções diferentes deram HWM total de 5614 µs e de 10557 µs. O HWM varia entre rodadas, por isso precisa de repetições e de medição na ESP32.
- Esses números **não são a medição oficial**. São só validação do código.

### 0.3 Decisões já tomadas (não reabrir sem motivo)
- Huffman em **um único header** (`huffman.h`), sem `.c` de lógica. Pedido explícito do usuário.
- Comentários em português, pensados para iniciante. Manter esse estilo.
- Uma parte por vez. Não avançar para a próxima sem confirmação do usuário.
- **Parte 2 (exportar dados da ESP32 para o PC) adiada** pelo usuário. Plano escolhido para quando voltar: saída serial em texto com marcadores (`INICIO`, uma linha por ativação, `FIM erros=...`), capturada por um script Python com pyserial.
- **Watchdog não mexido.** O usuário decidiu observar fisicamente primeiro. Discutir depois.
- **Build não mexido.** O usuário faz o `main/CMakeLists.txt`, `set-target`, `menuconfig` (240 MHz, -O2) pelo ESP-IDF.
- Não modificar arquivos sem pedido explícito. Quando o usuário pedir só informação, só informar.

### 0.4 Pendências técnicas conhecidas
- **Memória:** ~200 KB estáticos em `exp`. Pode não caber na SRAM livre da ESP32. Se der erro no build ou na execução, reduzir `MAX_ATIVACOES` ou remover `etapas` na primeira rodada. Decisão em aberto.
- **Recursão no `huffman.h`:** profundidade da árvore igual ao número de níveis. Com texto aleatório fica em torno de 10 a 12. Pior caso (frequências tipo Fibonacci) pode chegar a centenas de níveis, o que estoura a pilha de 8 KB. Não afeta o experimento atual.
- **Aviso de compilação no PC:** `huffman_validar defined but not used` em `experimento.c`. Não é erro.
- **Compilação no PC:** usar `gcc -std=gnu11 -Wall -Wextra`. Com `-std=c99`, `clock_gettime` e `struct timespec` não compilam.
- O ESP-IDF não está instalado nesta máquina. Nada de build para a ESP32 foi testado aqui.

### 0.5 Status do passo a passo (seção 12)
- Passos 1 e 2: **pendentes**. Dependem do usuário (prazo e ambiente ESP-IDF).
- Passo 3: **feito** (`huffman.h`).
- Passo 4: **parcial**. Medição e buffer prontos. Exportação CSV pendente (parte 2 adiada).
- Passo 5: **feito só no PC**. Falta rodar na ESP32 e ajustar N.
- Passos 6 a 15: **pendentes**.

### 0.6 Próximo passo previsto
1. Usuário compila e roda `experimento.c` na ESP32 pelo ESP-IDF. Relatar: erro de memória? HWM? Erros = 0?
2. Se houver erro de memória, decidir a redução do buffer (0.4).
3. Depois, passo 6 (definir D = HWM(100%) da etapa 1 × 1,2) e a parte 2 (exportação).

---

## 1. Objetivo e pergunta de pesquisa

Medir a mesma tarefa (codificação de Huffman) em quatro escopos com estresse crescente, na ESP32, e responder:

> Se a cobertura de código chega a 100%, por que o HWM continua crescente? E, quando o WCRT não é alcançável, como o modelo (m,k)-firme transforma as medições em um critério testável?

Ideias centrais do enunciado:
- Cobertura de 100% garante testar a lógica, não garante o WCRT. O pior tempo depende do hardware e do instante em que as tarefas competem.
- HWM (High Water Mark = maior tempo observado) é uma aproximação otimista do WCRT. A margem de segurança depende da variabilidade das medições.
- Quando o WCRT não é alcançável, a tolerância a perdas de deadline é especificada com o percentual de deadlines cumpridos, o skip factor S e o (m,k)-firme.

---

## 2. Tarefa sob teste

- **Algoritmo:** codificação de Huffman sobre um array de caracteres **aleatório**.
- **Linguagem:** C, no ESP-IDF.
- **Medição:** `clock_gettime(CLOCK_MONOTONIC)`, resolução em µs.
- **Validação obrigatória:** cada ativação codifica, decodifica e compara com a entrada. Se a comparação falhar, a ativação é contabilizada como erro e o experimento é marcado como inválido.
- **Registros obrigatórios:** tamanho N do array e semente do gerador aleatório. Tempo de resposta de **cada** ativação.
- **Referência do enunciado (não é nossa medição):** Linux em i7, 1.000 medições, mín 2378 µs, média 2493 µs, HWM 3200 µs. Usar apenas como comparação de ordem de grandeza no relatório.

### Ponto de partida no repositório (atualizado)
O ponto de partida original (`codificacao_huffman.c`) foi substituído. Já estão feitos:
- Lógica em `main/huffman.h` (sem `printf`, sem `esp_timer`).
- Medição com `clock_gettime(CLOCK_MONOTONIC)` em `main/experimento.c`.
- Array aleatório com semente registrada e tamanho N configurável (`gerar_texto`).
- Laço de N ativações com buffer estático, sem imprimir durante a medição.

Ver seção 0 para o estado atual.

---

## 3. Plataforma de medição (ESP32)

Descrever no relatório:

| Item | Valor (a preencher/confirmar) |
|---|---|
| Placa | ESP32 (modelo e revisão, ex.: ESP32-WROOM-32) |
| CPU | Xtensa LX6 dual-core |
| Núcleos usados | 1 (core fixado com `xTaskCreatePinnedToCore`) |
| Frequência | 240 MHz fixa, sem DFS (`CONFIG_PM_ENABLE` desligado). Esse é o equivalente ao governor |
| SO / RTOS | FreeRTOS (ESP-IDF), versão a registrar |
| Memória | SRAM interna, heap configurado |
| Wi-Fi / BT | Desligados nas etapas 1 a 3. Na etapa 4 só o tráfego de estresse está ativo |
| Otimização | `-O2` fixo, registrado |
| Tarefas em execução | Listar (IDLE, timer, tarefas do experimento) |
| Alimentação / temperatura | Registrar (fonte USB, temperatura se possível) |

Regra: **registrar as condições do teste em cada coleta**, porque mudar programas em execução muda a carga e o tempo medido.

---

## 4. Definição de tempo de resposta e de deadline

- **Etapa 1 (tarefa isolada, sem evento):** tempo de resposta = tempo de execução da Huffman, medido de início a fim da ativação.
- **Etapas 2, 3 e 4 (tarefas disparadas por evento ou periódicas):** tempo de resposta = do **instante de liberação** (timestamp da ISR ou da release periódica) até o fim da tarefa. Medir só o tempo de execução subestimaria a resposta.
- **Deadline D (fixo):** `D = HWM(100%) da etapa 1 × 1,2`. Definido **uma única vez** e usado nas quatro etapas.
- Ativação perde o deadline se `resposta > D`.
- Experimento D é a única exceção: varia D para comparar.

---

## 5. Etapas do experimento

| Etapa | Escopo | O que implementar na ESP32 | Estresse |
|---|---|---|---|
| 1 | Tarefa | Huffman isolada, com validação, laço de N ativações | Nenhum |
| 2 | Comportamental | Tarefa disparada por evento externo, um por vez. Evento por GPIO (ISR → semáforo binário → tarefa) | Um evento de cada vez, disparado por segundo dispositivo ou gerador de sinal |
| 3 | Intertarefas | 2 a 3 tarefas periódicas (`vTaskDelayUntil`) com taxas diferentes, mutex protegendo buffer compartilhado, Huffman incluída | Contenção, bloqueio e preempção |
| 4 | Sistema | Todas as anteriores juntas, com carga externa | Carga de CPU no outro core, tráfego UDP de um PC (iperf) e escrita em flash/NVS. **Justificar no relatório** |

Sugestão de tarefas da etapa 3 (a confirmar): sensor simulado a 10 ms, logger a 50 ms e Huffman com período próprio. Prioridades distintas, registradas no relatório.

### Caso, procedimento e critério de cada etapa (para o relatório)
Para cada etapa descrever:
- **Caso de teste:** entradas (N, semente, taxas, eventos), restrições e comportamento esperado.
- **Procedimento:** passos (flashar, configurar, disparar, coletar, exportar CSV).
- **Critério:** o que é aprovado (validação OK em todas as ativações, N ≥ 1.000, condições registradas).

Dica do enunciado: em tempo real o critério é buscar o pior caso de concorrência de hardware e de interrupções, tentando fazer o sistema perder o prazo.

---

## 6. Implementação (estrutura proposta)

```
atividade_m3/
├── plano_atividade.md          # este arquivo
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild       # etapa (1 a 4), N, semente, D, período das tarefas
│   ├── huffman.c/.h            # lógica pura: contagem, árvore, codificação, decodificação
│   ├── experiment.c/.h         # clock_gettime, buffer estático, exportação CSV pela serial
│   └── stage_*.c               # etapas 1 a 4
├── host/                       # testes unitários e gcov (somente no PC, para cobertura)
├── analysis/
│   └── analyze.py              # estatísticas, gráficos, janela deslizante, experimentos A a D
├── resultados/                 # CSV por etapa e cenário (ativacao, tempo_us)
├── relatorio/                  # PDF final
└── README.md                   # como compilar, coletar e analisar
```

### 6.1 Núcleo da tarefa (`huffman.c`)
- Separar lógica pura de I/O.
- Gerador de entrada: PRNG com semente registrada (ex.: xorshift32), gera N caracteres.
- Validação: decodificar e comparar byte a byte.
- Liberar toda a memória de cada ativação. Monitorar o heap mínimo com `esp_get_minimum_free_heap_size()` ao longo de 10.000 ativações, para detectar fragmentação.
- Tabela de códigos com `calloc`/`free` por ativação, como o código atual. Checar falha de alocação.

### 6.2 Instrumentação (`experiment.c`)
- `clock_gettime(CLOCK_MONOTONIC, &ts)` em volta da ativação, convertido para µs.
- Timestamp da ISR (etapa 2) com a mesma fonte de tempo, para a resposta incluir o atraso até a tarefa começar.
- Buffer estático de 10.000 × `uint32_t` (40 KB, cabe na SRAM).
- **Não imprimir durante a medição.** Gravar o buffer e só depois exportar o CSV pela serial (`ativacao,tempo_us`). Usar 921600 baud ou USB-Serial-JTAG.
- Desligar o WDT da tarefa de medição, se necessário, e documentar.
- Posicionar funções críticas em IRAM (`IRAM_ATTR`) na ISR, para não depender do cache de flash durante a medição.

### 6.3 Seleção de etapa (`Kconfig.projbuild`)
- Um único binário, com etapa escolhida no `menuconfig`.
- D em um único header/Kconfig, para não haver risco de variar entre etapas.

### 6.4 Cobertura (experimento A)
- Compilar a Huffman no PC com `gcc --coverage` (sem `ESP_PLATFORM`), com testes em `host/`: texto vazio, um símbolo, 256 símbolos, texto longo, validação de erro.
- Medir linha e desvio com `gcov --branch-probabilities` ou `gcovr`.
- Justificar o que ficar sem cobertura (ex.: caminho de falha de `calloc`, que exige injeção de falha).
- Medir o HWM da etapa 1 no momento em que a cobertura chega a 100% e comparar com as etapas 2, 3 e 4.
- Observação: a cobertura é medida no PC, mas o HWM é medido na ESP32. Deixar isso explícito no relatório.

### 6.5 Análise (`analysis/analyze.py`)
- Entrada: CSV de `resultados/`.
- Saídas:
  - Estatísticas: mín, média, HWM(100%), HWM(99%), % no prazo D.
  - Histograma com HWM, D e cauda marcados.
  - Gráfico temporal (resposta × ativação) com picos destacados.
  - Lista de perdas, distâncias entre perdas consecutivas, S e rajadas.
  - Janela deslizante k = 10, 20 e 50, com perdas por janela e pior caso, `m = k − perdas_máximas`.
  - Experimentos A, B, C e D.
  - Tabela-síntese.
- Bibliotecas: numpy, pandas, matplotlib. Fixar sementes e dimensões das figuras.

---

## 7. Análises obrigatórias (para cada etapa)

**6.1 Estatísticas simples e HWM**
- Mín, média, HWM(100%) e HWM(99%) (máximo sem o 1% das maiores medições).
- % de ativações que cumpriram D.

**6.2 Histograma e gráfico temporal**
- Histograma com HWM, D e a cauda da distribuição.
- Gráfico temporal (resposta × ativação), destacando picos.
- Comentar: onde está a maioria das medições? A cauda é longa e fina? Os picos são isolados ou agrupados?

**6.3 Distâncias entre perdas e skip factor**
- Listar as ativações com perda de D.
- Distâncias entre perdas consecutivas (ex.: 119, 116, 16, 2, 154...). Distância 1 = perdas em ativações consecutivas.
- Skip factor S = menor distância entre duas perdas.
- Rajadas (sequências de perdas) e a maior delas.

**6.4 Janela deslizante e (m,k)-firme (obrigatório)**
- Uma tarefa é (m,k)-firme se cumpre pelo menos m deadlines em qualquer janela de k ativações consecutivas, com m ≤ k.
- Para k = 10, 20 e 50: percorrer a sequência, contar perdas por janela e plotar.
- Pior caso de cada k e `m = k − perdas_máximas`.
- Informar o (m,k) da etapa. Exemplo da aula: pior janela com 6 perdas em 20 ativações → (14,20).
- Atenção: o (m,k) medido descreve o comportamento observado. **Não é garantia formal.** Discutir esse limite.

---

## 8. Experimentos obrigatórios

### Experimento A: limites da cobertura (etapa 1)
- Cobertura de linha e de desvio (gcov no PC).
- Criar casos até 100% (ou justificar).
- Registrar HWM da etapa 1 nesse ponto e comparar com etapas 2, 3 e 4.
- Discutir: a cobertura saturou, mas o HWM continuou subindo? O que isso mostra sobre "cobrir 100% dos caminhos não garante encontrar o WCRT"?

### Experimento B: convergência do HWM
- Traçar HWM acumulado × número de amostras: 10, 20, 100, 1.000 e 10.000.
- Pelo menos 10.000 ativações em um cenário.
- Discutir: com quantas amostras o HWM estabiliza? Houve saltos tardios? Com 10 ou 20 amostras, o sistema pareceria mais rápido do que é?

### Experimento C: margem de segurança
- Margens de 10%, 25% e 50% sobre o HWM da etapa 1.
- Comparar com o HWM da etapa 4.
- Discutir: em que etapa a margem deixa de ser suficiente? Margem fixa é adequada quando a variabilidade muda?
- Relacionar com a variação observada na própria ESP32 (ex.: faixas de µs medidas por nós, em vez de 12–14 ms vs 10–16 ms da aula, que foram de outra plataforma).

### Experimento D: perdas de deadline e (m,k) sob diferentes D
- Repetir 6.3 e 6.4 para pelo menos 3 valores de D: HWM(100%) da etapa 1, HWM(99%) da etapa 1 e um intermediário.
- Para cada D: % cumprido, S e (m,k) com k = 20.
- Encontrar o menor D que satisfaz um (m,k) exigido na etapa 4 e comparar com D = HWM(100%).
- Classificar as perdas em isoladas ou em rajada. Discutir por que, em sistema com inércia física, importa o tempo entre perdas consecutivas e não só o percentual de perdas.

---

## 9. Tabela-síntese (seção 8 do enunciado)

Com D fixo, preencher para as quatro etapas:

| Etapa | Cobertura (linha / desvio) | Mín (µs) | Média (µs) | HWM100 (µs) | HWM99 (µs) | % no prazo | S | (m,k) k=10 | (m,k) k=20 | (m,k) k=50 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1. Tarefa | | | | | | | | | | |
| 2. Comportamental | | | | | | | | | | |
| 3. Intertarefas | | | | | | | | | | |
| 4. Sistema | | | | | | | | | | |

---

## 10. Erros a evitar

- Medir só a tarefa isolada e concluir que o sistema é rápido (o "erro clássico" da aula).
- Alterar o D entre as etapas e comparar resultados.
- Usar poucas amostras (10 ou 20), que escondem a cauda.
- Reportar só média e máximo, sem gráficos e histogramas.
- Afirmar que o (m,k) medido é garantia.
- Não registrar as condições do teste (programas em execução, frequência, carga).
- Escolher margens de segurança "chutadas", sem relacioná-las à variabilidade medida.
- Imprimir na serial durante a medição (distorce o tempo).
- Medir na ESP32 com Wi-Fi ativo nas etapas 1 a 3 sem registrar.

---

## 11. Relatório e entregáveis

**Relatório em PDF (5 a 8 páginas):**
1. Identificação (nomes e RA) e descrição da plataforma (seção 3 deste plano).
2. Metodologia: caso, procedimento e critério de cada etapa, e justificativa do D.
3. Tabela-síntese e gráficos (histogramas, temporais, convergência do HWM e janela deslizante).
4. Respostas e discussões de A, B, C e D.
5. Seção específica sobre (m,k)-firme, incluindo limitações.
6. Ameaças à validade:
   - Plataforma: ESP32 com FreeRTOS, não é sistema de propósito geral. Ainda assim há jitter de tick, ISRs do sistema e Wi-Fi/BT se ativos.
   - Estados ocultos: cache de flash e de instruções, aquecimento da placa, estado da SRAM.
   - Interrupções do SO (FreeRTOS, tick, timers).
   - Efeito da instrumentação (`clock_gettime`, buffer, exportação).
   - Cenários de estresse não cobertos.
   - Cobertura medida no PC, HWM medido na ESP32.
7. Conclusão.

**Código-fonte** da Huffman, instrumentação e scripts de análise, com README curto para reproduzir.

**Dados brutos:** CSV de cada etapa e cenário (`ativacao, tempo_us`).

**Entrega:** um `.zip` nomeado com os RAs (ex.: `STR_M3_RA1_RA2.zip`). Um integrante envia; todos os nomes constam no relatório, junto com o link do repositório da solução.

---

## 12. Passo a passo de execução

1. **Confirmar prazo** com os professores (ver topo). **[PENDENTE]** O prazo indicado (12/11) pode já ter passado.
2. **Preparar ambiente ESP-IDF** e confirmar que `idf.py build flash monitor` funciona na placa. **[PENDENTE]** Usuário faz pelo ESP-IDF.
3. **Refatorar** `codificacao_huffman.c` para lógica pura + validação + gerador com semente. **[FEITO]** Está em `main/huffman.h` (header único, decisão do usuário).
4. **Implementar instrumentação** com `clock_gettime(CLOCK_MONOTONIC)`, buffer e exportação CSV pós-medição. **[PARCIAL]** Medição e buffer feitos. Exportação CSV pendente (parte 2, adiada).
5. **Etapa 1:** rodar, validar, ajustar N para a ordem de grandeza desejada. Gerar CSV. **[PARCIAL]** Feito só no PC (N=1000, 0 erros). Falta na ESP32, ajuste de N e CSV.
6. **Definir D** = HWM(100%) etapa 1 × 1,2. Congelar em um único lugar.
7. **Experimento A:** cobertura no PC (`host/`). Registrar HWM no ponto de 100%.
8. **Etapa 2:** ligar evento GPIO, medir resposta a partir da ISR. Gerar CSV.
9. **Etapa 3:** criar tarefas periódicas, mutex, Huffman. Gerar CSV.
10. **Etapa 4:** carga externa completa. Justificar o cenário. Gerar CSV.
11. **Experimento B:** rodar 10.000 ativações em uma etapa e gerar a curva de convergência.
12. **Analisar** com `analysis/analyze.py`: estatísticas, gráficos, janela deslizante, experimentos C e D.
13. **Preencher** a tabela-síntese.
14. **Escrever** o relatório (5 a 8 páginas) e o README.
15. **Montar o zip** e enviar pelo AVA.

---

## 13. Decisões em aberto (confirmar antes de começar)

1. **Prazo:** o enunciado diz 12/11 e hoje é 16/11/2026. Confirmar a data real.
2. **Modelo da ESP32** e se há um segundo dispositivo (ou gerador de sinal) para o evento externo da etapa 2.
3. **Segunda configuração (opcional):** como a plataforma de comparação não é Linux, a opção é repetir na mesma ESP32 com outra configuração (ex.: tarefa no core 0 vs core 1, ou com Wi-Fi ativo vs inativo). Confirmar se o professor aceita isso como "segunda plataforma".
4. **Tamanho N e período das tarefas:** definir depois da primeira medição da etapa 1.
5. **Carga da etapa 4:** confirmar que o grupo pode usar tráfego de rede de um PC como estresse.
