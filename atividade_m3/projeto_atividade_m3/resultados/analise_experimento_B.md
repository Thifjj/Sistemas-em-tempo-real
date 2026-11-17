# Experimento B: convergência do HWM na etapa 4 (sistema)

Análise feita só com a saída da primeira rodada do experimento B (`main/experimento_b.c`, N = 10000 ativações, tamanho = 1000 caracteres, semente 12345). Nenhuma rodada nova.

## 1. Condições do teste

- Etapa 4 (sistema): sensor (10 ms), Huffman periódica medida (30 ms), evento por timer (20 ms) e carga de ~50% no core 0.
- ESP-IDF v6.1, ESP32 (modelo 1), 2 núcleos, 240 MHz, -O2, power management desligado.
- Prioridades: sensor 6, Huffman 5, evento 4, carga 1.
- Erros de validação: 0.
- Deadline fixo: D = 13519 µs (= 11266 × 1,2, definido na etapa 1).

## 2. Resultados

| Item | Valor (µs) |
|---|---|
| Mínimo | 10773 |
| Média | 11725,4 |
| HWM | 13464 |
| Margem até o D (13519 − 13464) | 55 (0,4% de D) |

Perdas de deadline: **nenhuma**. 10000 de 10000 ativações (100%) cumpriram o prazo. Skip factor S: n/a. Maior rajada: 0. (m,k): (10,10), (20,20) e (50,50), sem perdas em nenhuma janela.

## 3. Convergência do HWM

| Amostras | HWM acumulado (µs) |
|---|---|
| 10 | 12484 |
| 20 | 12484 |
| 100 | 13032 |
| 1000 | 13464 |
| 10000 | 13464 |

- O HWM subiu até a amostra 1000 e não mudou entre 1000 e 10000. Ou seja, o pior caso observado nesta rodada apareceu dentro das primeiras mil ativações, e as outras 9000 não o superaram.
- Isso não garante que o pior caso real tenha sido visto: 10000 amostras só mostram um estado do sistema nesta execução.

## 4. Comparação com a etapa 4 (N = 1000)

- As rodadas de 1000 ativações da etapa 4 (`etapa4_teste.md`) deram HWM entre 13473 e 13498 µs e média entre 11728,5 e 11730,5 µs.
- Esta rodada deu HWM 13464 µs e média 11725,4 µs. Os valores são muito próximos, com HWM ligeiramente menor, dentro da variação entre rodadas.
- O resultado confirma que a etapa 4 fica consistentemente perto do D, a cerca de 21 a 55 µs dele.

## 5. Duração

- Cada ativação espera 30 ms antes de medir, e a resposta média é de 11,7 ms. Cada ciclo fica perto de 41,7 ms.
- Com 10000 ativações, a rodada leva cerca de 417 s (≈ 7 min). Esta é uma estimativa a partir da média; o log não traz o tempo de parede total.

## 6. Leitura para o relatório

- Nesta rodada, a etapa 4 cumpriu o D em todas as 10000 ativações, sem erros de validação.
- A margem até o D é de apenas 55 µs. Uma rodada um pouco pior pode gerar perdas, como já discutido em `analise_experimento_C.md`, seção 5.
- Na rodada B, o HWM ficou estável a partir de 1000 amostras. Porém, as duas rodadas do experimento D (mesma configuração, `resultado_exp_d.md`) continuaram subindo entre 1000 e 10000 amostras. Com este código, não dá para afirmar que 1000 amostras bastam.

## 7. Ressalvas

- Esta é a primeira rodada. Ainda não há repetições do experimento B para comparar a variação entre execuções.
- O HWM é o máximo observado, não o pior caso real.
- A carga está no core 0, junto com as Huffman, que não têm core fixo (ver `etapa4_teste.md`, seção 7).
- O CSV (`ativacao, tempo_us`) não foi exportado, então não há HWM(99%) nem histograma.
