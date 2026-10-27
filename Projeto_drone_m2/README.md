# Drone didático com ESP32 e FreeRTOS

Este projeto simula partes de um piloto automático para estudar tarefas de tempo real, comunicação Wi-Fi e medições temporais. O firmware é escrito em C com ESP-IDF; os programas em Python permitem receber registros e enviar comandos pelo computador. Os estados e as saídas dos motores são simulados: o código não controla um drone real.

## O que falta implementar

O trabalho ainda não está completo. A lista abaixo reúne as principais implementações necessárias conforme o enunciado e o código atual:

- **Fusão inercial:** implementar um estimador, como um filtro complementar, usando amostras simuladas. Hoje a tarefa apenas modifica os valores do estado.
- **Controle de atitude:** completar o controlador PID com os termos integral e derivativo; a implementação atual usa somente o termo proporcional.
- **Navegação:** implementar a atualização de waypoint ou rota. Atualmente o evento de navegação apenas executa uma carga simulada.
- **Fail-safe:** implementar a redução imediata do throttle nas saídas simuladas e manter um estado seguro que impeça o controle normal de sobrescrevê-las durante a emergência.
- **Associação de eventos e timestamps:** incluir o instante de cada evento na própria mensagem ou em outro mecanismo que preserve sua associação com a execução correspondente. Hoje NAV, FS e CTRL usam timestamps globais do evento mais recente, que podem ser sobrescritos. Contabilizar também eventos perdidos ou acumulados.
- **Métricas temporais:** calcular ou disponibilizar uma análise dos tempos de resposta por tarefa, incluindo WCRT observado, WCET observado, HWM, jitter, latência de ativação e ocupação de CPU. Completar os contadores de execuções e perdas de prazo de `MONITOR_TASK` e `TIME_TASK`.
- **Modelo `(m,k)-firm`:** implementar a análise de janelas de `k` execuções consecutivas para verificar quantas respeitaram o prazo em tarefas soft, como `NAV_PLAN`.
- **Cenários de teste:** preparar configurações reproduzíveis para as políticas de prioridade e os cenários de escalonamento e frequência escolhidos a partir dos resultados da M1, incluindo os testes em unicore. As prioridades atuais são fixas no código.
- **Análise da comunicação:** consolidar os dados de UDP e TCP para comparar RTT e impacto nos prazos, identificando protocolo e tamanho dos pacotes. Garantir que as medições entre dispositivos considerem a sincronização dos relógios e as perdas de registros.

Além da implementação, é preciso executar os cenários e testes, investigar as perdas de prazo observadas, analisar interferência e bloqueio, e preparar o relatório solicitado com evidências. A existência das métricas não garante, por si só, que os prazos sejam atendidos.

## Organização

- `main/main.c`: inicialização, tarefas, estado simulado, comunicação entre tarefas, medições e processamento dos comandos.
- `main/tcp_udp_esp32_wifi_con.h`: configuração e inicialização da rede Wi-Fi, endereços e portas.
- `main/udp_server.py`: cliente no PC para receber logs UDP e enviar comandos.
- `main/tcp_client.py`: cliente no PC para receber logs TCP e enviar comandos.
- `main/CMakeLists.txt`: fontes e dependências do componente ESP-IDF. O firmware compila `main.c`; `esp32_sntp_con.c` não está incluído na compilação atual.
- `sdkconfig` e `sdkconfig.defaults`: configuração do ESP-IDF; o padrão define o tick do FreeRTOS em 1000 Hz.

## Inicialização e comunicação entre tarefas

Em `app_main`, o firmware inicializa a memória NVS, configura o GPIO 2 como saída e cria o temporizador que apaga o LED. Em seguida inicializa o Wi-Fi, cria as filas e o semáforo e inicia as tarefas. O laço principal imprime periodicamente contadores acumulados de execuções e perdas de prazo.

A fila `qNav` entrega eventos à tarefa de navegação; o semáforo `semFS` sinaliza uma emergência. A fusão acorda o controle usando uma notificação de tarefa. A fila `qLog` armazena registros para envio pelo monitor, sem fazer as tarefas esperarem pela rede; registros rejeitados quando ela está cheia são contabilizados.

O tick configurado é de 1 ms. A tarefa de fusão tem período de 5 ms e o código exige uma frequência de pelo menos 200 Hz para que esse período não seja convertido em zero ticks.

## Tarefas

| Tarefa | Ativação e prazo | Funcionamento atual |
|---|---|---|
| `FUS_IMU` | Periódica, a cada 5 ms; prazo de 5 ms | Atualiza os valores simulados de roll, pitch e yaw, executa uma carga artificial de aproximadamente 1 ms e notifica o controle. |
| `CTRL_ATT` | Notificada pela fusão; prazo de 5 ms após o evento | Calcula erros de atitude e usa ganhos proporcionais para atualizar quatro saídas simuladas de motor. Também executa uma carga artificial de aproximadamente 0,8 ms. |
| `NAV_PLAN` | Recebe um evento pela fila; prazo de 20 ms | Simula processamento de navegação por aproximadamente 3,5 ms. Para eventos de telemetria, imprime o estado e simula uma carga menor. |
| `FS_TASK` | Acordada por semáforo; prazo de 10 ms | Executa uma carga artificial de aproximadamente 0,9 ms e registra o atendimento da emergência. A redução de throttle está indicada como simulação, mas não altera as saídas dos motores. |
| `MONITOR_TASK` | Periódica, a cada 500 ms | Gerencia sockets, recebe comandos, responde ao PC e envia registros disponíveis. Avisa na serial se a duração medida do ciclo exceder 500 ms. |
| `TIME_TASK` | Periódica, a cada 1 s após receber IP | Inicializa a sincronização SNTP e conta os ciclos desde a última atualização recebida. |

As cargas artificiais usam `cpu_tight_loop_us`, que mantém a CPU ocupada pelo intervalo indicado; os tempos observados podem variar com interrupções e escalonamento. As tarefas possuem prioridades definidas em `main.c` e são criadas com `xTaskCreate`.

## Rede e comandos

O Wi-Fi opera em modo estação. Configure `WIFI_SSID` e `WIFI_PASS` em `main/tcp_udp_esp32_wifi_con.h`. Após uma desconexão, o código informa o motivo e tenta reconectar; quando recebe um IP, libera o uso da rede pelas tarefas. Evite compartilhar credenciais reais no repositório.

O monitor aceita os comandos `NAV` e `FS` tanto por UDP quanto por TCP. Ele responde com `ACK` e o resultado: `OK`, `FILA_CHEIA`, `PENDENTE` ou `COMANDO_INVALIDO`. O comando `FS` acende o LED no GPIO 2 por um segundo e sinaliza a tarefa de emergência; outro comando `FS` reinicia a contagem do LED. O comando `NAV` coloca um evento na fila de navegação.

O UDP usa a porta 6010. Para receber seus logs, configure `PC_IP` com o endereço do computador na rede. O servidor TCP da ESP32 usa a porta 5000 e envia logs pela conexão do cliente. Ambos os sockets são não bloqueantes e são recriados após a reconexão Wi-Fi.

## Registros e medições

As tarefas registram os timestamps do evento, início e fim em microssegundos desde a inicialização. Com eles é possível calcular tempo de resposta (`fim - evento`), duração observada (`fim - início`) e latência de ativação (`início - evento`). As tarefas FUS, CTRL, NAV e FS mantêm contadores de execuções e de perdas de prazo; mensagens na serial exibem parte desses dados.

O monitor envia registros JSON com sequência, hora civil em milissegundos, estado da sincronização SNTP, ciclos desde a sincronização, nome da tarefa, timestamps e quantidade acumulada de registros perdidos. Os scripts do PC gravam registros em arquivos JSONL e calculam o RTT de comandos reconhecidos usando um relógio monotônico local. Esse RTT inclui espera e processamento, além da rede. Para comparar horários entre dispositivos, ambos os relógios precisam estar sincronizados.

Os timestamps dos eventos NAV, FS e CTRL são variáveis compartilhadas que guardam apenas o evento mais recente. Eventos próximos podem sobrescrever um timestamp anterior; portanto, a associação e as medições de cada evento não estão garantidas sob acúmulo. O projeto ainda não calcula WCRT, HWM, jitter, ocupação de CPU ou janelas `(m,k)-firm` de forma consolidada.

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
