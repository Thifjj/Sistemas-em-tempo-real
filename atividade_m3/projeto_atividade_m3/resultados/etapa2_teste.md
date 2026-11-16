# Etapa 2 (comportamental): registro do teste na ESP32

Este arquivo registra o procedimento, as condições e os resultados da etapa 2 (plano_atividade.md, seções 3, 4 e 5). Arquivo do firmware: `main/experimento_etapa2.c`.

## 1. Caso de teste

- **Tarefa:** codificação de Huffman (construir árvore, gerar códigos, codificar, decodificar) sobre um texto aleatório de 1000 caracteres.
- **Evento externo:** um timer one-shot de 20 ms, armado a cada ativação. Quando dispara, guarda a hora e libera a tarefa por semáforo. É uma **interrupção simulada**, aceita pelo enunciado. Só um evento acontece por vez.
- **Entradas:** N = 1000 ativações, tamanho do texto = 1000 caracteres, semente base = 12345 (semente de cada ativação = 12345 + i).
- **Validação:** cada ativação decodifica e compara com a entrada com `strcmp`. Critério de aprovação: **0 erros** em todas as ativações.
- **Medição:** `clock_gettime(CLOCK_MONOTONIC)`, resolução em µs. A resposta vai do instante em que o timer disparou até o fim da Huffman. A geração da entrada e a liberação de memória ficam fora do tempo medido.
- **Pausa entre ativações:** `vTaskDelay(1)` (1 tick, fora do tempo medido), para a task IDLE rodar e alimentar o watchdog.

## 2. Procedimento

1. `main/CMakeLists.txt` com `SRCS "experimento_etapa2.c"`.
2. `idf.py build flash monitor` com o `sdkconfig` configurado (240 MHz, -O2).
3. Copiar a saída completa do monitor para a seção 4.
4. Repetir 3 vezes, reiniciando a placa entre as rodadas (botão EN ou reset).

## 3. Condições do teste

Bloco impresso automaticamente pelo firmware no início de cada rodada:

- Etapa: 2 (Comportamental)
- ESP-IDF: v6.1
- Chip: ESP32 (modelo 1), 2 núcleos, revisão v3.1
- CPU: 240 MHz (configuração fixa)
- Power management: desligado (frequência fixa, equivale ao governor)
- Otimização: -O2 (`CONFIG_COMPILER_OPTIMIZATION_PERF`)
- Tarefas do FreeRTOS: 7
- Núcleo da tarefa de medição: 0 (a tarefa não tem core fixo)
- Wi-Fi: componente compilado no `sdkconfig`, **não iniciado pelo programa**
- Atraso do evento: 20000 µs

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
| 1 | 0 | 0 | 10766 | 11165.6 | 11458 |
| 2 | 0 | 0 | 10766 | 11165.6 | 11458 |
| 3 | 0 | 0 | 10766 | 11165.6 | 11458 |

Observação: as rodadas 1, 2 e 3 têm saída idêntica em todos os campos. Conferir se cada rodada é uma execução nova e não cópia da anterior antes de usar os dados.

Perdas de deadline (D = 13519 µs), iguais nas três rodadas:
- Ativações que perderam o prazo: nenhuma
- Cumpriram o prazo: 1000 de 1000 (100%)
- Perdas: 0. Skip factor S: n/a. Maior rajada: 0
- (m,k): (10,10), (20,20) e (50,50), pois não houve perdas

## 5. Comparação com a etapa 1

- HWM total: etapa 1 = 11266 µs; etapa 2 = 11458 µs (diferença de 192 µs).
- A etapa 2 é um pouco mais lenta porque a resposta inclui o disparo do timer e a troca de tarefa pelo semáforo, que a etapa 1 não tem.
- O D foi fixado a partir da etapa 1 (13519 µs) e **não foi alterado** nesta etapa.

## 6. Ressalvas para o relatório

- O evento é um timer do ESP-IDF, executado numa tarefa de alta prioridade, e não uma interrupção de hardware. Isso adiciona um pouco de latência ao disparo.
- A etapa 2 não tem a quebra por etapa interna da Huffman, só o total. Ver seção 6 do plano.
- O CSV (`ativacao, tempo_us`) não foi exportado, por decisão do grupo. Sem ele não há HWM(99%) nem histograma.
