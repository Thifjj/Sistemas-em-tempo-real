# Estimativa de WCRT com medições em ESP32: codificação de Huffman em quatro escopos

Disciplina: Sistemas em Tempo Real (UNIVALI, Eng. de Computação)
Professores: Michael Douglas C A e Felipe Viel
Integrantes: [nome e RA de cada integrante]
Repositório: [link]

> **RASCUNHO.** Seções marcadas com [PENDENTE] dependem de trabalho ainda não feito (ver seção 8).

## 1. Objetivo

Medir a mesma tarefa, a codificação de Huffman sobre um texto aleatório, em quatro escopos com estresse crescente, em uma ESP32 física. A pergunta é: o HWM (maior tempo observado) é uma boa estimativa do WCRT? E, quando o WCRT não é alcançável, como o modelo (m,k)-firme transforma as medições em um critério testável?

## 2. Plataforma

| Item | Valor |
|---|---|
| Placa | ESP32 (modelo 1), 2 núcleos, revisão v3.1 |
| Frequência | 240 MHz fixa. Power management desligado (equivale ao governor) |
| RTOS | FreeRTOS, ESP-IDF v6.1 |
| Otimização | -O2 (`CONFIG_COMPILER_OPTIMIZATION_PERF`) |
| Wi-Fi | Componente compilado, não iniciado pelo programa (etapas 1 a 3). Etapa 4 usa carga de CPU, sem rede |
| Medição | `clock_gettime(CLOCK_MONOTONIC)`, resolução em µs |

## 3. Tarefa sob teste e validação

- Tarefa: construir árvore, gerar códigos, codificar e decodificar um texto de 1000 caracteres (caracteres de 1 a 255).
- Entrada gerada por xorshift32, com semente registrada (12345).
- Validação: cada ativação decodifica e compara com a entrada (`strcmp`). Critério: 0 erros.
- A geração da entrada e a liberação de memória ficam fora da medição.

## 4. Definição de resposta e deadline

- Etapa 1: resposta = tempo de execução da Huffman.
- Etapas 2 a 4: resposta = do instante de liberação (evento ou `vTaskDelay`) até o fim da Huffman.
- **D = HWM(100%) da etapa 1 × 1,2 = 11266 × 1,2 = 13519 µs.** Definido uma vez e usado nas quatro etapas.

## 5. Escopos e resultados

| Etapa | Escopo | Estresse | N | Erros | Média (µs) | Mín (µs) | HWM (µs) | Perdas (D = 13519) |
|---|---|---|---|---|---|---|---|---|
| 1 | Tarefa isolada | Nenhum | 1000 | 0 | 10977,4 | 10582 | 11266 | 0 |
| 2 | Comportamental | Evento por timer de 20 ms, uma ativação por evento | 1000 | 0 | 11165,6 | 10766 | 11458 | 0 |
| 3 | Intertarefas | Sensor a 10 ms (prio 6), mutex, Huffman a 30 ms (prio 5) | 1000 | 0 | ~11236 | 10726 | 11527 | 0 |
| 4 | Sistema | Etapa 3 + evento a 20 ms (prio 4) + carga de ~50% no core 0 (prio 1) | 1000 | 0 | ~11729 | 10919 | 13473 a 13498 | 0 |

Observações:
- Etapa 1 foi repetida 3 vezes; as rodadas 1 e 2 têm saída idêntica em todos os campos. [PENDENTE: confirmar se é execução nova.]
- Etapa 3 rodou 3 vezes, com médias diferindo em ≤ 0,6 µs (execuções independentes).
- Etapa 4 rodou 3 vezes (N = 1000) com watchdog desligado. Uma rodada com o watchdog ligado foi descartada: os backtraces impressos durante a medição distorceram o resultado (HWM 38060 µs).

## 6. Experimento B: convergência do HWM (etapa 4, N = 10000)

| Amostras | 10 | 20 | 100 | 1000 | 10000 |
|---|---|---|---|---|---|
| HWM acumulado (µs) | 12484 | 12484 | 13032 | 13464 | 13464 |

- Mínimo 10773 µs, média 11725,4 µs, HWM 13464 µs, 0 erros, 100% das 10000 ativações no prazo.
- Nesta rodada, o HWM ficou estável a partir da amostra 1000. Porém, as duas rodadas do experimento D, com o mesmo código, continuaram subindo entre 1000 e 10000 amostras (ver seção 8). Por isso, a convergência não pode ser afirmada. Com 10 ou 20 amostras, o HWM ficou cerca de 1 ms abaixo do valor observado no longo prazo.
- Margem até D: 55 µs. Uma rodada um pouco pior pode gerar perdas.
- Detalhes em `resultados/analise_experimento_B.md`.

## 7. Experimento C: margem de segurança

| Margem sobre HWM da etapa 1 | Valor (µs) | Cobre etapa 4 (13498)? | Folga |
|---|---|---|---|
| 10% | 12392,6 | não (falta 1105 µs) | — |
| 25% | 14082,5 | sim | 585 µs |
| 50% | 16899,0 | sim | 3401 µs |

- A margem de 10%, que seria a usada com o D atual, não cobre a etapa 4.
- A margem de 25% cobre todas as etapas, mas a folga na etapa 4 é pequena.
- O HWM sobe quase 20% da etapa 1 para a etapa 4. Uma margem fixa não acompanha a variabilidade da concorrência.
- Detalhes em `resultados/analise_experimento_C.md`.

## 8. Experimentos ainda não concluídos

- **Experimento A (cobertura):** não realizado. Decisão do grupo: todas as medições são feitas só na ESP32, então não há cobertura de linha e de desvio (gcov) no PC. Por isso, não é possível discutir se a cobertura saturou enquanto o HWM continuou subindo. A discussão sobre "cobrir caminhos não garante o WCRT" fica apoiada só nas etapas 1 a 4.
- **Experimento D (perdas de deadline para diferentes D):** realizado em duas rodadas (etapa 4, N = 10000), com os tempos de resposta mantidos em memória e recalculados para D = 11266, 12393 e 13519 µs, além de uma varredura de 11000 a 14000 µs. Resultados em `resultados/resultado_exp_d.md`. D = 11266 µs deixa 15% no prazo; D = 12393 µs deixa cerca de 90%; D = 13519 µs não teve perdas nas duas rodadas.
- **HWM(99%), histograma e gráfico temporal:** não realizados pelo mesmo motivo (memória da ESP32).
- **Análise de (m,k)-firme com janela deslizante (k = 10, 20, 50):** [PENDENTE] só existe para a etapa 4, N = 10000 (experimento B). Nas demais etapas não há perdas, então (m,k) = (k,k).

## 9. Tabela-síntese (D = 13519 µs)

| Etapa | Mín (µs) | Média (µs) | HWM100 (µs) | HWM99 (µs) | % no prazo | S | (m,k) k=10 | (m,k) k=20 | (m,k) k=50 |
|---|---|---|---|---|---|---|---|---|---|
| 1. Tarefa | 10582 | 10977,4 | 11266 | [PENDENTE] | 100% | n/a | (10,10) | (20,20) | (50,50) |
| 2. Comportamental | 10766 | 11165,6 | 11458 | não medido | 100% | n/a | (10,10) | (20,20) | (50,50) |
| 3. Intertarefas | 10726 | ~11236 | 11527 | [PENDENTE] | 100% | n/a | (10,10) | (20,20) | (50,50) |
| 4. Sistema (N = 1000) | 10919 | ~11729 | 13498 | [PENDENTE] | 100% | n/a | (10,10) | (20,20) | (50,50) |
| 4. Sistema (N = 10000) | 10773 | 11725,4 | 13464 | [PENDENTE] | 100% | n/a | (10,10) | (20,20) | (50,50) |

Cobertura de código (linha / desvio): não medida (ver experimento A, seção 8).

## 10. Discussão

- **Por que o HWM cresce mesmo com a lógica testada?** A Huffman é a mesma em todas as etapas. O aumento do HWM vem da concorrência: troca de contexto, mutex, sensor e carga de CPU disputam o processador. Cobrir todos os caminhos de código não garante encontrar o pior caso de concorrência.
- **A etapa 4 está perto do deadline.** O HWM (13498 µs) fica a 21 µs de D, e a rodada de N = 10000 fica a 55 µs. Com outra rodada o HWM pode passar de D.
- **Core 0 saturado.** A carga e as duas Huffman (periódica e por evento) disputam o core 0. Na rodada com watchdog ligado, a IDLE0 não chegou a rodar. Isso indica que a carga poderia ficar no core 1.
- **Tempo entre perdas.** Não houve perdas nas etapas 1 a 4 nas rodadas válidas, nem no experimento B (D = 13519 µs). Por isso, a discussão de rajadas e skip factor fica sem caso real; só há a análise do critério.

## 11. Ameaças à validade

- Plataforma: ESP32 com FreeRTOS não é sistema de propósito geral. O tick do FreeRTOS é de 10 ms, o que quantiza os instantes de liberação.
- Estados ocultos: cache de flash e de instruções, aquecimento da placa e estado da SRAM não foram controlados.
- Instrumentação: `clock_gettime` e a cópia do buffer entram na medição. A exportação foi descartada para não imprimir durante a medição.
- Etapa 2 usa um timer do ESP-IDF em tarefa de alta prioridade, não uma interrupção de hardware. Isso adiciona latência ao disparo.
- Watchdog de tarefas desligado na etapa 4. Como o programa termina sozinho, não houve travamento, mas a condição é documentada.
- Carga simulada de CPU, sem tráfego de rede. O Wi-Fi não foi usado nesta versão.
- Cobertura de código não foi medida: todas as medições são feitas só na ESP32, sem gcov no PC.

## 12. Conclusão

[PENDENTE: escrever após fechar os experimentos A e D.]

Com os dados atuais: o HWM da etapa 1 (11266 µs) com margem de 20% dá D = 13519 µs, que é suficiente para as etapas 1 a 3 e apertado para a etapa 4. A margem de 10% não cobre a etapa 4. O HWM não convergiu para um valor fixo: na rodada B ele ficou estável a partir de 1000 amostras, mas nas duas rodadas do experimento D continuou subindo até 10000 amostras.

## Referências de dados

- `resultados/etapa1_teste.md`, `etapa2_teste.md`, `etapa3_teste.md`, `etapa4_teste.md`
- `resultados/analise_experimento_B.md`, `resultados/analise_experimento_C.md`
- Código: `main/experimento_etapa1.c` a `experimento_etapa4.c`, `experimento_b.c`, `huffman.h`
