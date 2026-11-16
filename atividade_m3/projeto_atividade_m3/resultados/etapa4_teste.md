# Etapa 4 (sistema): registro do teste na ESP32

Este arquivo registra o procedimento, as condições e os resultados da etapa 4 (plano_atividade.md, seções 3, 4 e 5). Arquivo do firmware: `main/experimento_etapa4.c`.

## 1. Caso de teste

Todas as partes rodam ao mesmo tempo:

- **Sensor** (prioridade 6): a cada 10 ms escreve um texto novo no buffer compartilhado, com caracteres de 1 a 255 (mesma regra da etapa 1).
- **Huffman periódica** (prioridade 5): a cada 30 ms copia o buffer, sob mutex, e codifica. **É a tarefa medida.**
- **Evento** (prioridade 4): um timer dispara um evento a cada 20 ms (interrupção simulada, como na etapa 2). A tarefa de evento faz uma Huffman por evento. **Não é medida**: só ocupa a CPU.
- **Carga** (prioridade 1, fixada no core 0): ocupa cerca de 50% do core 0 (3 ms ocupado, 3 ms livre).

- **Entradas:** N = 1000 ativações medidas, tamanho = 1000 caracteres, semente do sensor = 12345.
- **Validação:** cada ativação medida decodifica e compara com a entrada com `strcmp`. Critério: **0 erros**.
- **Medição:** `clock_gettime(CLOCK_MONOTONIC)`. A resposta vai do instante em que a Huffman periódica foi liberada (logo após o `vTaskDelay`) até o fim da codificação.
- **Duração:** cerca de 30 s por rodada.

## 2. Procedimento

1. `main/CMakeLists.txt` com `SRCS "experimento_etapa4.c"`.
2. `idf.py build flash monitor` com o `sdkconfig` configurado (240 MHz, -O2).
3. **Watchdog de tarefas desligado** no `sdkconfig` (`CONFIG_ESP_TASK_WDT_EN` não definido). Ver seção 6.
4. Copiar a saída completa do monitor para a seção 4.
5. Repetir 3 vezes, reiniciando a placa entre as rodadas.

## 3. Condições do teste

Bloco impresso automaticamente pelo firmware no início de cada rodada:

- Etapa: 4 (Sistema)
- ESP-IDF: v6.1
- Chip: ESP32 (modelo 1), 2 núcleos, revisão v3.1
- CPU: 240 MHz (configuração fixa)
- Power management: desligado (frequência fixa, equivale ao governor)
- Otimização: -O2 (`CONFIG_COMPILER_OPTIMIZATION_PERF`)
- Tarefas do FreeRTOS: 7
- Núcleo da tarefa de medição: 0 (a tarefa não tem core fixo)
- Wi-Fi: componente compilado no `sdkconfig`, **não iniciado pelo programa**
- Períodos: sensor 10 ms, Huffman 30 ms, evento 20 ms
- Prioridades: sensor 6, Huffman 5, evento 4, carga 1
- Carga: ~50% do core 0
- **Watchdog de tarefas: desligado** (alteração feita no `sdkconfig` para as rodadas válidas)

Condições anotadas à mão (preencher em cada rodada):

| Item | Valor |
|---|---|
| Fonte de alimentação | |
| Temperatura aproximada da placa | |
| Programas abertos no PC durante o teste | |
| Data e hora | |

## 4. Resultados por rodada (watchdog desligado)

| Rodada | Erros | Mín total (µs) | Média total (µs) | HWM total (µs) | Perdas (D = 13519 µs) |
|---|---|---|---|---|---|
| 1 | 0 | 10919 | 11728,5 | 13473 | 0 (100%) |
| 2 | 0 | 10919 | 11730,5 | 13498 | 0 (100%) |
| 3 | 0 | 10920 | 11729,4 | 13488 | 0 (100%) |

Perdas, iguais nas três rodadas:
- Ativações que perderam o prazo: nenhuma
- Skip factor S: n/a. Maior rajada: 0
- (m,k): (10,10), (20,20) e (50,50), pois não houve perdas

Observação: as médias diferem em no máximo 2 µs e o HWM varia 25 µs entre as rodadas. Isso indica execuções independentes, não cópias.

## 5. Comparação com as etapas 1, 2 e 3

| Etapa | Média (µs) | HWM (µs) | Perdas em D = 13519 µs |
|---|---|---|---|
| 1. Tarefa | ~10977 | 11266 | 0 |
| 2. Comportamental | 11165,6 | 11458 | 0 |
| 3. Intertarefas | ~11236 | 11527 | 0 |
| 4. Sistema | ~11729 | 13473 a 13498 | 0 |

- A etapa 4 tem média ~490 µs acima da etapa 3 e HWM ~1950 µs acima. O efeito vem da carga de CPU e da Huffman por evento, que disputam o processador com a Huffman medida.
- **A margem até o D é pequena.** O maior HWM (13498 µs) fica a **21 µs** do D (13519 µs). Com outra rodada, o HWM pode passar do D. Isso precisa ser discutido no relatório.
- O D foi fixado a partir da etapa 1 (13519 µs) e **não foi alterado**.

## 6. Rodada com o watchdog ligado (descartada, usar como evidência)

Esta rodada foi feita **antes** de desligar o watchdog de tarefas. Não entra na tabela de resultados válidos.

- O watchdog disparou várias vezes com a `IDLE0` sem rodar, e a saída mostra backtraces impressos **durante a medição**.
- Os backtraces atrasam as ativações. O HWM foi de 12997 µs para 13410 µs e depois saltou para **38060 µs** no fim da rodada, com **872 perdas** do D. Esse salto coincide com os backtraces, então é artefato da própria depuração, não do sistema.
- Ainda assim, mostra o problema de fundo: o core 0 ficou sem a `IDLE0` rodar, ou seja, saturado. A carga (~50%) somada às duas Huffman (periódica e por evento) passa de 100% do core 0.

Resultado dessa rodada (só para registro): média 11745,1 µs, mín 10864 µs, HWM 38060 µs, 0 erros, 1 perda isolada no fim (99,9% no prazo).

## 7. Ressalvas para o relatório

- **Watchdog de tarefas desligado** nas rodadas válidas. Isso remove a proteção contra travamento. Como o programa termina sozinho, não houve travamento, mas a condição precisa aparecer no relatório.
- **Core 0 saturado:** a tarefa de carga está fixada no core 0, junto com as Huffman, que não têm core fixo. A `IDLE0` não chegou a rodar na rodada com o watchdog ligado. Vale discutir se a carga devia ficar no core 1, como o plano sugere ("carga no outro core").
- **Carga simulada:** é só de CPU (3 ms ocupado / 3 ms livre). O enunciado cita tráfego de rede como exemplo; Wi-Fi/UDP não entrou nesta versão.
- **Evento por timer:** é uma interrupção simulada, executada numa tarefa de alta prioridade do ESP-IDF, não numa ISR de hardware.
- **Medição com `vTaskDelay`:** o período é contado a partir do fim da tarefa anterior, então o período real fica um pouco acima do nominal.
- O CSV (`ativacao, tempo_us`) não foi exportado, por decisão do grupo. Sem ele não há HWM(99%) nem histograma.
