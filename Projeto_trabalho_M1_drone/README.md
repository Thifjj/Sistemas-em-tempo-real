# Drone ESP32 com Touch Sensor e FreeRTOS

Este projeto demonstra tarefas concorrentes no ESP32 usando ESP-IDF 6.1. O sensor touch aciona eventos de navegação, telemetria e fail-safe. A IMU, o controlador de atitude e as saídas dos motores são **simulados**; o programa não controla motores reais.

O código principal está em [`main/touch_sens_basic_example_main.c`](main/touch_sens_basic_example_main.c).

## Visão geral

```text
FUS_IMU --notificação--> CTRL_ATT -- calcula --> saídas ESC simuladas

Touch CH4 --fila--> NAV_PLAN (evento de navegação)
Touch CH9 --fila--> NAV_PLAN (telemetria)
Touch CH0 --semáforo--> FS_TASK (fail-safe simulado)
```

As tarefas são fixadas no núcleo 0. Seus números de prioridade estão definidos no início do arquivo: quanto maior o número, maior a prioridade no FreeRTOS.

## Touches e ações

| Canal | GPIO mostrado na inicialização | Ação quando tocado |
| --- | --- | --- |
| CH4 | GPIO13 | Envia evento de navegação para `NAV_PLAN` |
| CH9 | GPIO32 | Envia evento de telemetria para `NAV_PLAN` |
| CH0 | GPIO4 | Acorda `FS_TASK` |
| CH3 | GPIO15 | Configurado e calibrado, mas sem ação associada |

Os GPIOs são consultados pela API do driver e impressos na inicialização. Confirme que correspondem à sua placa antes de conectar qualquer circuito.

## Partes do programa

### Configuração e recursos compartilhados

`FUS_T_MS` define o período pretendido da tarefa `FUS_IMU` (5 ms). `PRIO_*` define prioridades e `STK` define o tamanho de pilha passado ao criar cada tarefa.

`hFUS`, `hCTRL`, `hNAV` e `hFS` guardam os identificadores das tarefas. `qNav` é uma fila com capacidade para oito eventos de navegação ou telemetria. `semFS` é um semáforo binário para acordar a tarefa de fail-safe.

`g_state` guarda roll, pitch e yaw simulados. `g_esc` armazena os quatro percentuais simulados dos motores. A função `esc_write_simulated()` limita cada valor ao intervalo de 0 a 100%; ela não envia sinal elétrico a um ESC.

### Simulação de tempo

`cpu_tight_loop_us(us)` ocupa a CPU pelo tempo indicado, medido por `esp_timer_get_time()`. O código usa essa função para representar o custo de processamento da fusão, controle, navegação e fail-safe. Ela não mede o tempo real desses algoritmos, pois eles são apenas demonstrações.

### Tarefa `FUS_IMU`

Executa periodicamente. Atualiza os valores simulados de roll, pitch e yaw, consome aproximadamente 1 ms de CPU e notifica `CTRL_ATT`. `vTaskDelayUntil()` mantém o ritmo com base no período configurado.

O projeto usa `CONFIG_FREERTOS_HZ=1000` em [`sdkconfig.defaults`](sdkconfig.defaults), permitindo representar um período de 5 ms em ticks do FreeRTOS. Se o projeto já tiver um `sdkconfig` local, confirme nele que `CONFIG_FREERTOS_HZ` também está em `1000`.

### Tarefa `CTRL_ATT`

Fica bloqueada em `ulTaskNotifyTake()` até receber a notificação de `FUS_IMU`. Calcula erros entre a atitude simulada e as referências, aplica ganhos proporcionais simples (`kp_roll`, `kp_pitch`, `kp_yaw`) e mistura os resultados para obter `m1` a `m4`.

As saídas passam por `esc_write_simulated()`. A cada 100 execuções, os percentuais são impressos no monitor serial. A carga de aproximadamente 800 µs representa o processamento do controlador.

### Tarefa `NAV_PLAN`

Fica bloqueada em `xQueueReceive()` até chegar um evento. Para `EV_NAV`, imprime que recebeu navegação e simula 3,5 ms de processamento. Para `EV_TEL`, imprime roll, pitch e yaw e simula 500 µs de processamento.

### Tarefa `FS_TASK`

Fica bloqueada em `xSemaphoreTake()` até CH0 liberar o semáforo. Então simula 900 µs de ação e imprime o tempo decorrido. No código atual, essa ação **não reduz o throttle nem altera as saídas dos motores**; é somente uma demonstração do fluxo de fail-safe.

### Callbacks do touch

`example_touch_on_active_cb()` é chamado quando um canal é tocado. Como o callback pode rodar em contexto de interrupção, ele usa `xQueueSendFromISR()` para enviar eventos e `xSemaphoreGiveFromISR()` para sinalizar o fail-safe. O callback retorna se uma tarefa de prioridade maior foi acordada.

`example_touch_on_inactive_cb()` só registra no log qual canal foi liberado.

### Calibração e inicialização do touch

`example_touch_do_initial_scanning()` liga o sensor, faz leituras iniciais, lê o valor de referência (*benchmark*, quando suportado) e calcula o limite de ativação com `s_thresh2bm_ratio` (2%). A fórmula muda conforme a versão do hardware touch: no Touch V1 o valor cai ao tocar; nas versões seguintes o limite é calculado de outra forma. Depois, a configuração de cada canal é atualizada.

`example_touch_init()` cria o controlador e os quatro canais, configura o filtro, calibra, registra os callbacks, habilita o sensor e inicia a leitura contínua. `ESP_ERROR_CHECK()` interrompe a inicialização se uma chamada do ESP-IDF retornar erro.

### `app_main()`

Cria a fila e o semáforo, inicia as quatro tarefas e inicializa o touch. Depois mantém a tarefa principal viva com uma espera de um segundo.

## Compilar, gravar e acompanhar

No terminal configurado para ESP-IDF 6.1:

```sh
idf.py build
idf.py -p PORT flash monitor
```

Troque `PORT` pela porta serial da placa, por exemplo `/dev/ttyUSB0`. Para sair do monitor serial, pressione `Ctrl+]`.

Na saída serial, `ESC: ...` mostra os valores simulados dos motores; `TEL: ...` indica telemetria; `NAV_PLAN` indica um evento de navegação; e `FAIL-SAFE!` indica a execução simulada do tratamento de emergência.

## Limites desta demonstração

- Não há leitura de uma IMU real nem controle PID completo.
- As saídas ESC são apenas números impressos; não há PWM nem comunicação com controladores de motor.
- O fail-safe não altera as saídas dos motores.
- CH3 está configurado, mas não tem comportamento associado.
- Os estados compartilhados entre tarefas são demonstração e não usam proteção de concorrência.
