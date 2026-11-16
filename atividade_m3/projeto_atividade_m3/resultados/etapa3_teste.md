# Etapa 3 (intertarefas): registro do teste na ESP32

Este arquivo registra o procedimento, as condições e os resultados da etapa 3 (plano_atividade.md, seções 3, 4 e 5). Arquivo do firmware: `main/experimento_etapa3.c`.

## 1. Caso de teste

- **Tarefa medida:** Huffman periódica, codificando um texto de 1000 caracteres lido de um buffer compartilhado.
- **Concorrência:**
  - **Sensor** (prioridade 6): a cada 10 ms escreve um texto novo no buffer, com caracteres aleatórios de 1 a 255 (mesma regra da etapa 1, para a carga de trabalho ser igual).
  - **Huffman** (prioridade 5): a cada 30 ms copia o buffer, sob mutex, e faz a codificação completa.
- **Entradas:** N = 1000 ativações da Huffman, tamanho = 1000 caracteres, semente do sensor = 12345.
- **Validação:** cada ativação decodifica e compara com a entrada com `strcmp`. Critério de aprovação: **0 erros**.
- **Medição:** `clock_gettime(CLOCK_MONOTONIC)`, resolução em µs. A resposta vai do instante em que a Huffman foi liberada (logo após o `vTaskDelay`) até o fim da codificação. A cópia do buffer fica dentro do tempo medido.
- **Duração:** cerca de 30 s por rodada (1000 × 30 ms).

## 2. Procedimento

1. `main/CMakeLists.txt` com `SRCS "experimento_etapa3.c"`.
2. `idf.py build flash monitor` com o `sdkconfig` configurado (240 MHz, -O2).
3. Copiar a saída completa do monitor para a seção 4.
4. Repetir 3 vezes, reiniciando a placa entre as rodadas (botão EN ou reset).

## 3. Condições do teste

Bloco impresso automaticamente pelo firmware no início de cada rodada:

- Etapa: 3 (Intertarefas)
- ESP-IDF: v6.1
- Chip: ESP32 (modelo 1), 2 núcleos, revisão v3.1
- CPU: 240 MHz (configuração fixa)
- Power management: desligado (frequência fixa, equivale ao governor)
- Otimização: -O2 (`CONFIG_COMPILER_OPTIMIZATION_PERF`)
- Tarefas do FreeRTOS: 6
- Núcleo da tarefa de medição: 0 (a tarefa não tem core fixo)
- Wi-Fi: componente compilado no `sdkconfig`, **não iniciado pelo programa**
- Períodos: sensor 10 ms, Huffman 30 ms (`vTaskDelay`, não `vTaskDelayUntil`)
- Prioridades: sensor 6, Huffman 5

Condições anotadas à mão (preencher em cada rodada):

| Item | Valor |
|---|---|
| Fonte de alimentação | |
| Temperatura aproximada da placa | |
| Programas abertos no PC durante o teste | |
| Data e hora | |

## 4. Resultados por rodada

| Rodada | Núcleo | Erros | Mín total (µs) | Média total (µs) | HWM total (µs) |
|---|---|---|---|---|---|
| 1 | 0 | 0 | 10726 | 11236,4 | 11527 |
| 2 | 0 | 0 | 10726 | 11235,9 | 11526 |
| 3 | 0 | 0 | 10726 | 11236,5 | 11527 |

Observação: as médias das três rodadas diferem entre si (≤ 0,6 µs), e o HWM varia 1 µs. Isso indica execuções independentes, não cópias.

Perdas de deadline (D = 13519 µs), nas três rodadas:
- Ativações que perderam o prazo: nenhuma
- Cumpriram o prazo: 1000 de 1000 (100%)
- Perdas: 0. Skip factor S: n/a. Maior rajada: 0
- (m,k): (10,10), (20,20) e (50,50), pois não houve perdas

## 5. Comparação com as etapas 1 e 2

| Etapa | Média (µs) | HWM (µs) |
|---|---|---|
| 1. Tarefa | ~10977 | 11266 |
| 2. Comportamental | 11165,6 | 11458 |
| 3. Intertarefas | ~11236 | 11527 |

- A etapa 3 tem média e HWM um pouco maiores que a etapa 1 (+~260 µs na média e +261 µs no HWM). O efeito vem da concorrência com o sensor e do mutex, além da troca de contexto.
- A etapa 3 fica um pouco acima da etapa 2 (+~70 µs na média e +69 µs no HWM).
- O D foi fixado a partir da etapa 1 (13519 µs) e **não foi alterado**. A margem do D sobre o HWM da etapa 3 é de ~1990 µs.

## 6. Histórico de ajustes

1. **Primeira rodada invalidada:** o sensor escrevia só letras e espaço (27 símbolos). Isso deixava a Huffman bem mais rápida (~1880 µs), e a comparação com a etapa 1 ficava injusta. Os números dessa rodada **não** entram neste registro.
2. **Correção:** o sensor passou a usar caracteres de 1 a 255, a mesma regra do `huffman_gerar_texto` da etapa 1. Todas as rodadas acima foram feitas com essa versão.

## 7. Ressalvas para o relatório

- Os períodos são medidos com `vTaskDelay`, então cada período é contado a partir do fim da tarefa anterior, e o período real fica um pouco acima do nominal (ex.: 30 ms + tempo de execução). Isso deve ser descrito.
- O tick do FreeRTOS na ESP32 é de 10 ms por padrão. O despertar da tarefa só acontece em múltiplos de 10 ms, o que explica parte da variação do instante de liberação.
- O CSV (`ativacao, tempo_us`) não foi exportado, por decisão do grupo. Sem ele não há HWM(99%) nem histograma.
