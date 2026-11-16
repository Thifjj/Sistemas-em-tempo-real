# Etapa 1 (tarefa isolada): registro do teste na ESP32

Este arquivo registra o procedimento, as condições e os resultados da etapa 1 (plano_atividade.md, seções 3, 4 e 5).

## 1. Caso de teste

- **Tarefa:** codificação de Huffman (construir árvore, gerar códigos, codificar, decodificar) sobre um array de caracteres aleatório.
- **Entradas:** N = 1000 ativações, tamanho do texto = 1000 caracteres, semente base = 12345 (a semente de cada ativação é 12345 + i).
- **Validação:** cada ativação decodifica e compara com a entrada com `strcmp`. Critério de aprovação: **0 erros** em todas as ativações.
- **Medição:** `clock_gettime(CLOCK_MONOTONIC)`, resolução em µs. A geração da entrada e a liberação de memória ficam fora do tempo medido.
- **Pausa entre ativações:** `vTaskDelay(1)` (1 tick, fora do tempo medido), para a task IDLE rodar e alimentar o watchdog.

## 2. Procedimento

1. `idf.py build flash monitor` com o `sdkconfig` configurado (240 MHz, -O2).
2. Copiar a saída completa do monitor para o arquivo de rodadas (seção 4).
3. Repetir 3 vezes, reiniciando a placa entre as rodadas (botão EN ou reset).

## 3. Condições do teste

Bloco impresso automaticamente pelo firmware no início de cada rodada:

- ESP-IDF: v6.1
- Chip: ESP32 (modelo 1), 2 núcleos, revisão v3.1
- CPU: 240 MHz (configuração fixa)
- Power management: desligado (frequência fixa, equivale ao governor)
- Otimização: -O2 (`CONFIG_COMPILER_OPTIMIZATION_PERF`)
- Tarefas do FreeRTOS: 6
- Núcleo da tarefa de medição: registrado em cada rodada (a tarefa não tem core fixo)
- Wi-Fi: componente compilado no `sdkconfig`, **não iniciado pelo programa**

Condições anotadas à mão (preencher em cada rodada):

| Item | Valor |
|---|---|
| Fonte de alimentação | |
| Temperatura aproximada da placa | |
| Programas abertos no PC durante o teste | |
| Data e hora | |

## 4. Resultados por rodada

Preencher com a saída do monitor. Uma linha por rodada.

| Rodada | Núcleo | Erros | Mín total (µs) | Média total (µs) | HWM total (µs) | HWM criar árvore (µs) | HWM gerar códigos (µs) | HWM codificar (µs) | HWM decodificar (µs) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 0 | 0 | 10582 | 10977.4 | 11266 | 7919 | 1644 | 874 | 831 |
| 2 | 0 | 0 | 10582 | 10977.4 | 11266 | 7919 | 1644 | 874 | 831 |
| 3 | 0 | 0 | 10582 | 10977.7 | 11266 | 7919 | 1644 | 874 | 832 |

Observação: as rodadas 1 e 2 têm saída idêntica em todos os campos. Conferir se a rodada 2 é uma nova execução e não cópia da 1 antes de usar os dados.

## 5. Definição do D

Regra do plano: **D = HWM(100%) da etapa 1 × 1,2**, definido uma única vez e usado nas quatro etapas.

- HWM(100%) escolhido: maior HWM total entre as 3 rodadas = 11266 µs
- D = 11266 × 1,2 = 13519,2 µs (≈ 13519 µs)

D ainda **não está congelado**: falta confirmar a rodada 2 (ver observação na seção 4).

**Decisão: não usar CSV e não calcular HWM(99%) nem histograma.** O grupo decidiu não exportar as 1000 medições por CSV, e por isso o HWM(99%) e o histograma ficam fora do relatório. Pendência em aberto: o enunciado (seções 4, 6.1, 6.2 e 9) pede CSV, HWM(99%) e histograma, então isso precisa ser confirmado com os professores.

## 6. Histórico de ajustes no firmware

Registro dos problemas encontrados e das mudanças feitas. Os valores das rodadas com problema **não** entram no D.

1. **Erro de memória no link:** o buffer estático com 10000 ativações estourou a DRAM em 32528 bytes. Reduzido para `MAX_ATIVACOES = 5000`.
2. **Watchdog do IDLE1:** a tarefa de medição ocupava o core 1 sem ceder a CPU. Os backtraces impressos durante a medição distorceram os HWM (picos de 43 ms e 52 ms). Tentativa de desligar o WDT da tarefa não resolveu, porque o alerta era do IDLE.
3. **Watchdog do IDLE0:** com a tarefa solta, o scheduler passou a usá-la no core 0 e o mesmo problema apareceu. Solução: `vTaskDelay(1)` no fim de cada ativação, fora do tempo medido. A rodada seguinte saiu sem aviso do watchdog e sem picos.
4. **CSV descartado:** a exportação das 1000 medições pela serial não será feita (decisão do grupo). Ver seção 5.
5. **Core fixo removido:** a tarefa é criada com `xTaskCreate` (sem afinidade). O enunciado não exige core fixo, mas exige registrar o número de núcleos.

Observação: a rodada com `vTaskDelay(1)` e sem aviso de watchdog é a primeira considerada válida para o D.
