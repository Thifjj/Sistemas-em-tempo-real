# Drone ESP32 + FreeRTOS

Simulador didatico de drone para medir deadlines, comunicacao TCP/UDP e comportamento
real-time. Nao controla motores reais.

## Arquivos utilizados

- `main/main.c`: tarefas, sensores touch, rede, comandos e resumos.
- `main/drone_sim.h`: modelo, filtro, controle, navegacao e emergencia.
- `main/timing_metrics.h`: WCRT, HWM, misses e (9,10)-firm.
- `main/tcp_udp_esp32_wifi_con.h`: Wi-Fi Enterprise, enderecos e portas.
- `main/Kconfig.projbuild`: protocolo, identificador e limiar touch.
- `components/experiment_trace/`: hooks do escalonador e estatisticas off-CPU.
- `main/run_experiment.py`: ensaio automatizado e coleta.
- `main/summarize_experiments.py`: tabelas CSV/Markdown.
- `main/collect_logs.py`, `tcp_client.py`, `udp_server.py`: coleta manual.
- `tests/`: verificacoes locais do modelo e dos relatorios.
- `resultados/`: ensaios e resultados atuais.

## Compilar e gravar

Ative o ambiente ESP-IDF 6.1 e execute:

```bash
idf.py menuconfig
idf.py -p /dev/ttyUSB0 build flash monitor
```

Escolha protocolo em Drone experiment e frequencia em Component config / ESP System
Settings. Melhor: 160 MHz, preempcao 1; pior: 80 MHz, preempcao 0; slicing 1 e RM.
O SDK original fixa preempcao 1 em components/freertos/config/include/freertos/FreeRTOSConfig.h.
Alterar essa macro afeta tambem Wi-Fi e timers; o modo cooperativo exige validacao.
Nao desabilite watchdogs para transformar uma falha em resultado valido.

## Experimento

Com IP obtido na serial, execute a combinacao correspondente ao firmware:

```bash
python3 main/run_experiment.py --host 10.120.27.125 --protocol UDP --configuration melhor
```

Troque UDP por TCP e melhor por pior nas outras combinacoes. Cada execucao realiza
um teste fisico inicial e cinco repeticoes automaticas de aproximadamente 60 s.
A duracao real e os atrasos do roteiro ficam registrados.

Conecte eletrodos antes de ligar. No aviso, toque GPIO13/CH4 (NAV), GPIO32/CH9 (TEL),
GPIO4/CH0 (FS) e GPIO15/CH3 (TEL). Nao precisam ser simultaneos; ha 60 s para os quatro.
A latencia fisica e deteccao -> inicio da tarefa. Essa referencia unica e compartilhada
entre repeticoes. Depois o script desliga a varredura e injeta touches simulados.
O fail-safe zera os motores; START reinicia o estado entre repeticoes.

A ESP envia somente resumos a cada 500 ms, ACKs e snapshots TOUCHGET. Os resumos
incluem todas as execucoes nas metricas, mas preservam timestamps individuais apenas
da ultima e da pior execucao. HWM100=WCRT observado; HWM99 tem resolucao de 100 us,
com indicador de overflow. NAV_PLAN usa (9,10)-firm. Os demais contadores hard e soft
sao separados; qualquer miss impede declarar que todos os deadlines foram atendidos,
mas nao interrompe por si so o experimento.

O estado usa mutex com heranca de prioridade. Disputas, espera e observacoes de dono
com prioridade menor sao salvas por tarefa. Tempo off-CPU tambem inclui bloqueios;
nao reconstroi todas as preempcoes nem identifica sozinho a tarefa causadora.

Latencia Protocolo usa RTT maximo PC->ACK, com e sem reenvio separados. Nao e atraso
unidirecional. Para comparacoes entre dispositivos, verifique a sincronizacao do PC
e use --clock-synchronized. A ESP aguarda SNTP. TCP espera ACK por ate 60 s sem reenviar;
UDP repete a mesma sequencia e o firmware evita executar duplicatas.

## Saidas e testes

Cada pasta contem raw.jsonl, physical_touch.json e repetition_N.json. Em interrupcoes,
ha failure.json e resultado parcial quando aplicavel. Use --output com outra pasta
para nao misturar suites de versoes ou configuracoes diferentes.

```bash
python3 main/summarize_experiments.py resultados
python3 -m unittest discover -s tests -p 'test_experiment.py'
cc -std=c11 -Wall -Wextra -Werror -Imain tests/test_drone.c -lm -o /tmp/test_drone
/tmp/test_drone
```

As tabelas ficam em tabela.csv e tabela.md. Os testes locais nao substituem a validacao
na placa. Ha uma inconsistência conhecida nas liberacoes periodicas, por alinhamento
com os ticks, que pode subestimar tempos de resposta; valide antes da analise definitiva.
