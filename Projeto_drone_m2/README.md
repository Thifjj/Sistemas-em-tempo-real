# Drone didático com ESP32 e FreeRTOS

Este projeto simula partes de um piloto automático para estudar tarefas de tempo real, comunicação Wi-Fi e medições temporais. O firmware é escrito em C com ESP-IDF; os programas em Python permitem receber registros e enviar comandos pelo computador. Os estados e as saídas dos motores são simulados: o código não controla um drone real.

## O que ainda falta fazer

As funcionalidades e métricas abaixo já estão implementadas e passaram por testes no PC e compilação do firmware. A validação temporal na ESP32 ainda precisa ser realizada.

- Preparar os cenários reproduzíveis de melhor e pior atendimento da M1, com prioridades, frequência de CPU, preempção/time slicing e execução unicore conforme os cenários escolhidos. As prioridades atuais continuam fixas em `main.c`.
- Executar testes prolongados na placa, incluindo eventos próximos, saturação das filas, comandos de rede e emergência durante o controle. Investigar as perdas de prazo e verificar se a instrumentação atende aos prazos nos cenários exigidos.
- Comparar UDP e TCP em condições equivalentes, registrar tamanho dos pacotes e protocolo nos resultados e verificar a sincronização do relógio do PC. O RTT já é medido pelos clientes, mas inclui a espera pelo monitor de 500 ms.
- Melhorar o tratamento de sobrecarga e do fluxo TCP: filas limitadas podem rejeitar eventos e logs, e o receptor TCP atual aceita um comando por leitura, fechando a conexão quando chegam várias linhas na mesma leitura. Os clientes devem aguardar ACK antes de enviar o próximo comando nos testes atuais.
- Coletar evidências, analisar interferência e bloqueio, montar as tabelas comparativas e elaborar o relatório do trabalho. Percentis e janelas calculados no PC ficam incompletos quando há perda de logs; conferir também os resumos cumulativos do firmware.
- Se for necessário separar WCET de CPU da duração observada, acrescentar instrumentação por execução que desconte interrupções e preempções. `C_wall_max_us` mede duração de parede; a ocupação por core já usa os contadores de execução do FreeRTOS.

## Implementações realizadas

- Filtro complementar sobre giroscópio, acelerômetro e referência de yaw simulados, com bias e ruído determinístico.
- PID de roll, pitch, yaw e altitude, com integral limitada, anti-windup e saídas saturadas.
- Rota com quatro waypoints e atualização de referências de atitude e altitude a cada comando `NAV`.
- Emergência persistente que zera os quatro motores simulados e impede o rearme por controle ou navegação até reiniciar a placa.
- Eventos com ID, origem, timestamp e, no controle, cópia do estado da amostra. Filas preservam esses dados e registram rejeições.
- Métricas para as seis tarefas: execuções, misses, WCRT observado, maior duração observada, latência e desvio de início. HWM(100%) equivale ao WCRT observado; HWM(99%) tem estimativa por histograma no firmware e cálculo exato sobre os registros recebidos no PC.
- Modelo `(9,10)-firm` para `NAV_PLAN`, com janelas deslizantes e contagem de violações após completar dez execuções.
- Ocupação de CPU por core a partir da diferença dos contadores de execução das tarefas idle.

## Organização

- `main/main.c`: inicialização, tarefas, filas, integração da simulação, medições e comandos.
- `main/drone_sim.h`: modelo dinâmico didático, filtro complementar, PIDs, navegação e emergência.
- `main/timing_metrics.h`: acumuladores temporais, histograma e janelas `(9,10)-firm`.
- `main/analyze_logs.py`: análise dos registros JSONL no PC.
- `tests/test_drone.c`: testes locais do modelo e das métricas.
- `main/tcp_udp_esp32_wifi_con.h`: configuração e inicialização da rede Wi-Fi, endereços e portas.
- `main/udp_server.py`: cliente no PC para receber logs UDP e enviar comandos.
- `main/tcp_client.py`: cliente no PC para receber logs TCP e enviar comandos.
- `main/CMakeLists.txt`: fontes e dependências do componente ESP-IDF. O firmware compila `main.c`; `esp32_sntp_con.c` não está incluído na compilação atual.
- `sdkconfig` e `sdkconfig.defaults`: configuração do ESP-IDF; o padrão define o tick do FreeRTOS em 1000 Hz.

## Inicialização e comunicação entre tarefas

Em `app_main`, o firmware inicializa a memória NVS, configura o GPIO 2 como saída e cria o temporizador que apaga o LED. Em seguida inicializa o Wi-Fi, cria as filas de eventos e inicia as tarefas. O laço principal imprime periodicamente contadores acumulados de execuções e perdas de prazo.

As filas `qNav`, `qFS` e `qCtrl` entregam eventos completos às tarefas de navegação, emergência e controle. Cada evento inclui ID, origem e instante de geração; a fila do controle inclui também o estado produzido pela fusão. Os IDs correlacionam a fusão com o controle correspondente. As filas têm capacidade para 16 eventos, envio sem espera e contadores de rejeição por tarefa. Se um envio falhar, o evento não é contado como execução nem como miss: aparece separadamente em `drops`.

A fila `qLog` armazena até 1024 registros sem fazer as tarefas esperarem pela rede. Registros rejeitados são contabilizados em `perdidos`. Estado do modelo, métricas e contadores compartilhados possuem seções críticas curtas para acesso entre cores.

O tick configurado é de 1 ms. A tarefa de fusão tem período de 5 ms e o código exige uma frequência de pelo menos 200 Hz para que esse período não seja convertido em zero ticks.

## Tarefas

| Tarefa | Ativação e prazo | Funcionamento atual |
|---|---|---|
| `FUS_IMU` | Periódica, a cada 5 ms; prazo de 5 ms | Avança o modelo físico simplificado, funde medidas simuladas, executa carga artificial de aproximadamente 1 ms e envia a amostra ao controle. |
| `CTRL_ATT` | Evento da fusão recebido por fila; prazo de 5 ms após o evento | Calcula PIDs de atitude e altitude para atualizar quatro saídas simuladas de motor. Também executa uma carga artificial de aproximadamente 0,8 ms. |
| `NAV_PLAN` | Recebe um evento pela fila; prazo de 20 ms | Seleciona o próximo waypoint, calcula referências de roll/pitch a partir da posição e define altitude desejada; mantém carga de aproximadamente 3,5 ms. Para eventos de telemetria, imprime o estado e simula uma carga menor. |
| `FS_TASK` | Evento recebido por fila; prazo de 10 ms | Zera imediatamente os motores simulados e trava a emergência, limpa os integradores e executa carga artificial de aproximadamente 0,9 ms. O controle não pode reativar os motores. |
| `MONITOR_TASK` | Periódica, a cada 500 ms | Gerencia sockets, recebe comandos, responde ao PC e envia registros disponíveis. Avisa na serial se a duração medida do ciclo exceder 500 ms. |
| `TIME_TASK` | Periódica, a cada 1 s após receber IP | Inicializa a sincronização SNTP e conta os ciclos desde a última atualização recebida. |

As cargas artificiais usam `cpu_tight_loop_us`, que mantém a CPU ocupada pelo intervalo indicado; os tempos observados podem variar com interrupções e escalonamento. As tarefas possuem prioridades definidas em `main.c` e são criadas com `xTaskCreate`.

## Rede e comandos

O Wi-Fi opera em modo estação. Configure `WIFI_SSID` e `WIFI_PASS` em `main/tcp_udp_esp32_wifi_con.h`. Após uma desconexão, o código informa o motivo e tenta reconectar; quando recebe um IP, libera o uso da rede pelas tarefas. Evite compartilhar credenciais reais no repositório.

O monitor aceita os comandos `NAV` e `FS` tanto por UDP quanto por TCP. Ele responde com `ACK` e o resultado: `OK`, `FILA_CHEIA`, `PENDENTE` ou `COMANDO_INVALIDO`. O comando `FS` acende o LED no GPIO 2 por um segundo e envia um evento à tarefa de emergência; outro comando `FS` reinicia a contagem do LED. O comando `NAV` coloca um evento na fila de navegação.

O UDP usa a porta 6010. Para receber seus logs, configure `PC_IP` com o endereço do computador na rede. O servidor TCP da ESP32 usa a porta 5000 e envia logs pela conexão do cliente. Ambos os sockets são não bloqueantes e são recriados após a reconexão Wi-Fi.

## Registros e medições

As seis tarefas registram evento, início e fim em microssegundos desde o boot, ID do evento e origem (`0`: entrada local; `1`: PC; `2`: fusão; `3`: periódica). Para tarefas periódicas, o evento é a liberação planejada, incrementada pelo período; para o controle é o instante de entrega da amostra; para comandos é o instante em que o monitor os processa. A latência de comando medida na tarefa não inclui o tempo que ele aguardou no socket.

As métricas usam `fim - evento` como resposta, `fim - início` como duração observada e `início - evento` como latência de ativação. Os resumos `METRIC`, emitidos na serial a cada cinco segundos, incluem `runs`, `misses`, `drops`, `WCRT_us`, `C_wall_max_us`, `latency_max_us` e `jitter_max_us`. Este último é o maior desvio absoluto de início em relação à liberação, e tem interpretação de jitter para tarefas periódicas; para eventos é variação/atraso de despacho. As durações incluem preempções e interrupções; os máximos são observados, sem garantia analítica de pior caso.

O histograma cumulativo estima HWM(99%) em intervalos de 100 µs. `HWM99_bin_us` é o limite superior do intervalo, exceto quando `overflow=1`, que indica percentil na faixa acima de 100 ms. WCRT equivale a HWM(100%). Sem execuções, os indicadores de máximos não representam uma medição. O modelo `(9,10)-firm` avalia todas as janelas sobrepostas de dez execuções de `NAV_PLAN`, exigindo pelo menos nove respostas dentro de 20 ms; `windows` e `violations` permanecem zero durante o aquecimento. Eventos rejeitados são apresentados separadamente e não devem ser ignorados na avaliação final.

As linhas `CPU` apresentam ocupação por core no intervalo desde o resumo anterior (o primeiro cobre desde o boot), calculada como `100 × (1 - tempo_idle / tempo_total)` usando estatísticas do FreeRTOS com relógio `esp_timer`. Isso inclui tarefas do sistema e o custo da instrumentação. As linhas `STATE` mostram atitude, altitude, posição, waypoint, emergência e motores.

O monitor envia JSON com sequência, horário civil, sincronização SNTP, ciclos desde a sincronização, tarefa, ID, origem, timestamps e perdas de registros. Os clientes gravam arquivos JSONL e calculam RTT dos comandos pelo relógio monotônico do PC. Comparações de horários entre dispositivos exigem relógios sincronizados. Use um arquivo novo por boot/cenário.

Para obter WCRT, HWM(99%), duração máxima, latência, misses e `(9,10)-firm` dos registros recebidos:

```bash
python3 main/analyze_logs.py log_udp.jsonl
```

Também funciona com `log_tcp.jsonl`. Os percentis no PC são exatos somente sobre as amostras recebidas. Perdas de rede ou da fila invalidam a interpretação dessas janelas como todas as execuções consecutivas; os contadores locais de métricas continuam acumulando mesmo sem rede.

## Compilar e executar

Com o ambiente ESP-IDF 6.1 configurado e a placa conectada, compile, grave e abra o monitor serial:

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Substitua a porta caso necessário. A velocidade configurada do monitor serial é 115200 baud; use `Ctrl+]` para sair do monitor ESP-IDF.

Conecte o PC e a ESP32 à mesma rede. Anote o IP informado pela ESP32 na mensagem `IP obtido` e ajuste `ESP_IP` no script escolhido. Para UDP, ajuste também `PC_IP` no firmware para o IP do PC e grave novamente se ele mudar.

```bash
python3 main/udp_server.py
```

Ou, para TCP:

```bash
python3 main/tcp_client.py
```

Digite `NAV` ou `FS` no prompt e pressione Enter; digite `sair` para encerrar. Os scripts salvam os registros recebidos e enviados em `log_udp.jsonl` ou `log_tcp.jsonl` no diretório em que foram executados. No UDP, os eventos recebidos são gravados sem inundar o terminal; os ACKs são exibidos.

## Testes locais

Os testes verificam estabilidade da fusão, resposta de altitude, saturação e anti-windup, progressão de waypoints, persistência da emergência, máximos e limites das janelas `(9,10)-firm`. Execute:

```bash
cc -std=c11 -Wall -Wextra -Werror -Imain tests/test_drone.c -lm -o /tmp/test_drone
/tmp/test_drone
```

Esses testes validam o modelo e os cálculos no PC; os prazos e a comunicação precisam ser verificados na ESP32 após a gravação.
