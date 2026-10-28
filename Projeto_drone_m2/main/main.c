/*
 * Drone ESP32 + FreeRTOS
 * ESP-IDF 6.1
 * 1)
 * ----------------------------------------------------
 * Melhor atendimento de prazo:
 * Frequencia: 160mhz
 * Preempcao: 1
 * Slicing: 1
 * Politica: RM
 * ----------------------------------------------------
 * Pior atendimento de prazo:
 * Frequencia: 80
 * Preempcao: 0
 * Slicing: 1
 * Politica: RM
 * ----------------------------------------------------
 * 2)Comunicacao devje ser feita com UDP e TCP, deve identificar qual e o pior e o melhor protocolo para essa aplicacao (mostrando evidencias)
 * 3)Log deve capturar a hora atual capturada via SNTP (Pela TIME_TASK), MS no momento do envio pela esp e pc para analisar o delay da comunicao WIFI e o quanto ela compromete
 * o atendimento de restricoes de tempo, isso permitira calcular a RTT (Round Trip Time). "Para facilitar analise deve gerar um pacote com mensagens contendo o numero
 * de sequencia, protocolo utilizado, tamanho pacote, e os timestamps coletados via SNTP, assim podera medir o atraso de transmissao (latencia) entre os dispositivos"
 * 4)Medicao do WCRT (worst case response time) de cada tarefa/task, determinando o maior tempo de resposta observando durante o experimento, utiliza-se a metodologia HWM
 * (High water mark) na analise.
 * 5)O modelo (m,k)-firm será usado para avaliar o cumprimento parcial de deadlines em tarefas do tipo soft real-time.
 * Deve-se definir janelas de k execuções consecutivas e contar quantas delas (m) cumpriram seus prazos.
 * Por exemplo, um sistema (9,10)-firm garante que pelo menos 9 de cada 10 execuções respeitam o deadline.
 * 6)indicar quais tarefas sofrem interferencia e quais sofrem bloqueio (com base em mecanismos de inversao de prioridade)
 * 7)criterios de sucesso: atender todos deadlines durante varios testes
 * 8)Coletar o log timestamp_evento, timestamp_task_start, timestamp_task_end
 * calcular misses, latencia da comunicacao para ativar a task via comunicacao e ocupacao da cpu
 * tabela comparativa por cenario (politica x preemptividade x frequencia)
 */

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/touch_sens.h"
#include "touch_sens_example_config.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include <time.h>
#include "tcp_udp_esp32_wifi_con.h"

#include "drone_sim.h"
#include "timing_metrics.h"
#include "experiment_trace.h"

#ifndef CONFIG_DRONE_PROTOCOL_TCP
#define CONFIG_DRONE_PROTOCOL_TCP 0
#endif
#if CONFIG_DRONE_PROTOCOL_TCP
#define EXP_PROTOCOL "TCP"
#else
#define EXP_PROTOCOL "UDP"
#endif
#define EXP_META "\"experiment\":\"" CONFIG_DRONE_EXPERIMENT "\",\"protocol\":\"" EXP_PROTOCOL "\",\"policy\":\"RM\",\"frequency_mhz\":" STR(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) ",\"preemption\":" STR(configUSE_PREEMPTION) ",\"slicing\":" STR(configUSE_TIME_SLICING)
#define STR_(x) #x
#define STR(x) STR_(x)

#define TAG "DRONE"

// ======================================================
// PERIODICIDADE E PRIORIDADES
// ======================================================

#define FUS_T_MS 5

// RM: activation intervals 5 ms (fusion/control), 10 ms (monitor/FS),
// 20 ms minimum (navigation), 1000 ms (time); FS/navigation are sporadic.
#define PRIO_MON 4
#define PRIO_TIME 1
#define PRIO_FS_TASK 4
#define PRIO_FUS_IMU 6
#define PRIO_CTRL_ATT 5
#define PRIO_NAV_PLAN 3
#define BLINK_GPIO 2
#define STK 6144
// Network buffers plus STOP -> print_metrics require more stack than control tasks.
#define STK_MON 12288

#define D_FUS_IMU 5

#define D_CTRL_ATT 5

#define D_NAV_PLAN 20

#define D_FS_TASK 10

// Cada evento transporta timestamp, sequencia local e origem.
typedef enum
{
    SRC_TOUCH,
    SRC_PC,
    SRC_FUSION,
    SRC_PERIODIC,
    SRC_SIM_TOUCH
} event_source_t;
typedef struct
{
    int type;
    int64_t timestamp;
    uint32_t id;
    bool resource_contended, lower_priority_holder;
    int touch_channel;
    uint32_t command_seq;
    int64_t pc_send_ms, received_us, received_epoch_ms, lock_wait_us;
    event_source_t source;
    state_t sample;
} event_t;
static touch_sensor_handle_t physical_sensor;
static bool physical_scanning;
static volatile bool physical_touch_enabled;
static uint32_t physical_touch_seen;
static int64_t physical_latency[10];
static int64_t physical_event_time[10];
static uint32_t physical_done;
static uint32_t experiment_run;
static bool experiment_active = true;
static int64_t experiment_start;
static void print_metrics(bool verbose);
static uint32_t event_sequence;
static portMUX_TYPE event_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t new_event_id(void)
{
    portENTER_CRITICAL_SAFE(&event_lock);
    uint32_t id = ++event_sequence;
    portEXIT_CRITICAL_SAFE(&event_lock);
    return id;
}
static volatile uint32_t touches_nav, touches_tel, touches_fs;
static drone_t drone;
static SemaphoreHandle_t drone_lock;
static QueueHandle_t qCtrl, qFS;
static const char *metric_names[] = {"FUS_IMU", "CTRL_ATT", "NAV_PLAN", "FS_TASK", "MONITOR_TASK", "TIME_TASK"};
static const uint32_t deadlines[] = {5000, 5000, 20000, 10000, 500000, 1000000};
static timing_metrics_t metrics[6];
static uint32_t events_expected[6];
static portMUX_TYPE metric_lock = portMUX_INITIALIZER_UNLOCKED;
static void event_drop(unsigned task)
{
    portENTER_CRITICAL_SAFE(&metric_lock);
    if (experiment_active) metrics[task].drops++;
    portEXIT_CRITICAL_SAFE(&metric_lock);
}
// ======================================================
// HANDLES DAS TASKS
// ======================================================

static TaskHandle_t hFUS = NULL;
static TaskHandle_t hCTRL = NULL;
static TaskHandle_t hNAV = NULL;
static TaskHandle_t hFS = NULL;
static TaskHandle_t hMON = NULL;

// ======================================================
// IPC - COMUNICAÇÃO ENTRE TASKS
// ======================================================

// Eventos possíveis da task NAV_PLAN
typedef enum
{
    EV_NAV = 1,
    EV_TEL = 2
} nav_evt_t;

// Fila para NAV / Telemetria
static QueueHandle_t qNav = NULL;

static QueueHandle_t qSummary[7 + portNUM_PROCESSORS];
typedef struct
{
    char text[1400];
} summary_t;
static size_t packet_size(char *text, size_t capacity)
{
    size_t n = strlen(text);
    if (n < 2 || text[n - 2] != '}')
        return n;
    static uint32_t packet_sequence;
    unsigned long sequence=packet_sequence++;
    int tail = snprintf(text+n-2,capacity-n+2,",\"packet_seq\":%10lu,\"send_epoch_ms\":%20lld,\"bytes\":%6u}\n",sequence,0LL,0U);
    if (tail < 0 || (size_t)tail >= capacity-n+2) return 0;
    size_t total=n-2+tail;
    snprintf(text+n-2,capacity-n+2,",\"packet_seq\":%10lu,\"send_epoch_ms\":%20lld,\"bytes\":%6u}\n",sequence,0LL,(unsigned)total);
    return total;
}
static int64_t epoch_ms(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (int64_t)t.tv_sec * 1000 + t.tv_usec / 1000;
}
static void stamp_send(char *text) {
    char *field=strstr(text,"\"send_epoch_ms\":");
    if (field) { char value[21]; snprintf(value,sizeof(value),"%20lld",(long long)epoch_ms()); memcpy(field+16,value,20); }
}
static void enter_drone(event_t *event)
{
    int64_t t=esp_timer_get_time();
    if (xSemaphoreTake(drone_lock,0) != pdTRUE) {
        event->resource_contended=true;
        TaskHandle_t holder=xSemaphoreGetMutexHolder(drone_lock);
        if (holder && uxTaskPriorityGet(holder)<uxTaskPriorityGet(NULL)) event->lower_priority_holder=true;
        BaseType_t taken=xSemaphoreTake(drone_lock,portMAX_DELAY);
        assert(taken==pdTRUE);
    }
    event->lock_wait_us += esp_timer_get_time()-t;
}
static uint32_t logs_perdidos;
static portMUX_TYPE log_lock = portMUX_INITIALIZER_UNLOCKED;
static bool hora_sincronizada;
static esp_timer_handle_t led_timer;
static void apagar_led(void *arg)
{
    gpio_set_level(BLINK_GPIO, 0);
}
static uint32_t ciclos_sntp;
static portMUX_TYPE tempo_lock = portMUX_INITIALIZER_UNLOCKED;
static void registrar_evento(unsigned task, event_t event, int64_t inicio, int64_t fim)
{
    portENTER_CRITICAL(&metric_lock);
    if (!experiment_active || event.timestamp < experiment_start) {
        portEXIT_CRITICAL(&metric_lock);
        experiment_trace_end(task);
        return;
    }
    timing_add(&metrics[task], event.timestamp, inicio, fim, deadlines[task], task == 2);
    metrics[task].lock_wait_total_us += event.lock_wait_us;
    if ((uint64_t)event.lock_wait_us > metrics[task].lock_wait_max_us) metrics[task].lock_wait_max_us=event.lock_wait_us;
    if (event.resource_contended) metrics[task].lock_samples++;
    if (event.lower_priority_holder) metrics[task].lower_priority_holder++;
    if (event.source == SRC_SIM_TOUCH) metrics[task].simulated_touch_mask |= 1U << event.touch_channel;
    if ((event.source == SRC_PC || event.source == SRC_SIM_TOUCH) && event.pc_send_ms) {
        int64_t delay=(event.received_epoch_ms-event.pc_send_ms)*1000+inicio-event.received_us;
        if (!metrics[task].pc_samples || delay>metrics[task].pc_activation_max_us) metrics[task].pc_activation_max_us=delay;
        metrics[task].pc_samples++;
        metrics[task].last_pc_send_ms=event.pc_send_ms; metrics[task].last_rx_ms=event.received_epoch_ms;
        metrics[task].last_rx_us=event.received_us;
    }
    portEXIT_CRITICAL(&metric_lock);
    if (event.source == SRC_TOUCH) {
        portENTER_CRITICAL(&event_lock);
        physical_latency[event.touch_channel]=inicio-event.timestamp;
        physical_event_time[event.touch_channel]=event.timestamp;
        physical_done |= 1U << event.touch_channel;
        portEXIT_CRITICAL(&event_lock);
        ESP_LOGI(TAG,"TOUCH confirmado canal=%d task=%s latencia_us=%lld",event.touch_channel,metric_names[task],(long long)(inicio-event.timestamp));
    }
    experiment_trace_end(task);
}

// ======================================================
// FUNÇÃO PARA SIMULAR TEMPO DE EXECUÇÃO
// ======================================================

static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < us)
    {
        __asm__ __volatile__("nop");
    }
}

// ======================================================
// TASK 1 - FUSÃO IMU
// Periódica: 5 ms
// ======================================================

// HARD - PQ? depedendo do tempo de resposta desta task como sua funcao e produzir o estado inercial e importante que ele esteja correto para o controle utilizar
// e tambem notificar/acordar outra task de importancia essencial (CTRL_ATT) nao adianta acordar o controle com os dados ja incorretos para a acao
// T=5ms D=5ms C:falta medir tempo de computacao dela
static void task_fus_imu(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    int64_t release = esp_timer_get_time(), previous = release;
    for (;;)
    {
        int64_t start = esp_timer_get_time();
        float dt = limit((start - previous) / 1000000.0f, .001f, .05f);
        previous = start;
        event_t event = {.timestamp = release, .id = new_event_id(), .source = SRC_PERIODIC};
        experiment_trace_begin(0);
        enter_drone(&event);
        drone_fuse(&drone, dt);
        state_t sample = drone.estimate;
        xSemaphoreGive(drone_lock);
        cpu_tight_loop_us(1000);
        event_t control = {.timestamp = esp_timer_get_time(), .id = event.id, .source = SRC_FUSION, .sample = sample};
        if (xQueueSend(qCtrl, &control, 0) != pdTRUE)
            event_drop(1);
        registrar_evento(0, event, start, esp_timer_get_time());
        if (xTaskDelayUntil(&next, pdMS_TO_TICKS(FUS_T_MS)) == pdFALSE)
            vTaskDelay(1); // An overdue cycle must still let lower-priority tasks run.
        release += FUS_T_MS * 1000LL;
    }
}

// PID de atitude e altitude usa o estado da amostra que originou o evento.
static void task_ctrl_att(void *arg)
{
    event_t event;
    int64_t previous = 0;
    for (;;)
        if (xQueueReceive(qCtrl, &event, portMAX_DELAY) == pdTRUE)
        {
            int64_t start = esp_timer_get_time();
            experiment_trace_begin(1);
            float dt = previous ? limit((event.timestamp - previous) / 1000000.0f, .001f, .05f) : .005f;
            previous = event.timestamp;
            enter_drone(&event);
            drone_control(&drone, event.sample, dt);
            xSemaphoreGive(drone_lock);
            cpu_tight_loop_us(800);
            registrar_evento(1, event, start, esp_timer_get_time());
        }
}

static void task_nav_plan(void *arg)
{
    event_t event;
    for (;;)
        if (xQueueReceive(qNav, &event, portMAX_DELAY) == pdTRUE)
        {
            int64_t start = esp_timer_get_time();
            experiment_trace_begin(2);
            enter_drone(&event);
            if (event.type == EV_NAV)
                drone_navigate(&drone);
            state_t state = drone.estimate;
            xSemaphoreGive(drone_lock);
            if (event.type == EV_TEL)
                printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f altitude=%.2f\n",
                       state.roll, state.pitch, state.yaw, state.altitude);
            cpu_tight_loop_us(event.type == EV_NAV ? 3500 : 500);
            registrar_evento(2, event, start, esp_timer_get_time());
        }
}

static void task_fail_safe(void *arg)
{
    event_t event;
    for (;;)
        if (xQueueReceive(qFS, &event, portMAX_DELAY) == pdTRUE)
        {
            int64_t start = esp_timer_get_time();
            experiment_trace_begin(3);
            enter_drone(&event);
            drone_fail_safe(&drone);
            xSemaphoreGive(drone_lock);
            cpu_tight_loop_us(900);
            registrar_evento(3, event, start, esp_timer_get_time());
        }
}

// #
// TASK - 5 MONITOR_TASK
// D - 500ms
// Recebe comandos do pc para ativar as tasks FS_TASK e NAV_PLAN
// Requisito: HARD
// Motivo: Chama tasks com requisito hard e sua perca pode levar a uma mudanca atrasada ou uma falha de seguranca (FAIL SAFE)
// #

// Comandos aceitos em ambos os protocolos: NAV e FS (uma linha por comando).
// A resposta indica se o evento foi aceito pela fila.
static const char *execute_command(const char *cmd, int64_t received_us, int64_t received_ms)
{
    uint32_t command_seq = 0;
    long long pc_send = 0;
    char action[16], extra;
    if (sscanf(cmd, "%" SCNu32 " %lld %15s %c", &command_seq, &pc_send, action, &extra) != 3)
        return "COMANDO_INVALIDO\n";
    cmd = action;
    if (strcmp(cmd, "TOUCHGET") == 0) {
        static char result[384];
        portENTER_CRITICAL(&event_lock);
        uint32_t mask=physical_done;
        int64_t latency[4]={physical_latency[4],physical_latency[9],physical_latency[0],physical_latency[3]};
        int64_t times[4]={physical_event_time[4],physical_event_time[9],physical_event_time[0],physical_event_time[3]};
        portEXIT_CRITICAL(&event_lock);
        snprintf(result,sizeof(result),"OK TOUCH %lu %lld %lld %lld %lld %lld %lld %lld %lld\n", (unsigned long)mask,
                 (long long)latency[0],(long long)latency[1],(long long)latency[2],(long long)latency[3],
                 (long long)times[0],(long long)times[1],(long long)times[2],(long long)times[3]);
        return result;
    }
    if (strcmp(cmd, "TOUCHON") == 0) {
        if (!physical_sensor) return "TOUCH_NOT_READY\n";
        if (!physical_scanning) {
            if (touch_sensor_start_continuous_scanning(physical_sensor) != ESP_OK) return "TOUCH_START_FAILED\n";
            physical_scanning=true;
        }
        portENTER_CRITICAL(&event_lock);
        if (!physical_touch_enabled) { physical_touch_seen=0; physical_done=0; }
        physical_touch_enabled=true;
        portEXIT_CRITICAL(&event_lock);
        return "OK\n";
    }
    if (strcmp(cmd, "TOUCHOFF") == 0) {
        physical_touch_enabled=false;
        if (physical_scanning) {
            if (touch_sensor_stop_continuous_scanning(physical_sensor) != ESP_OK) return "TOUCH_STOP_FAILED\n";
            physical_scanning=false;
        }
        return "OK\n";
    }
    if (strcmp(cmd, "T4") == 0 || strcmp(cmd, "T9") == 0 || strcmp(cmd, "T0") == 0 || strcmp(cmd, "T3") == 0) {
        int channel=cmd[1]-'0';
        event_t ev = {.type=channel == 4 ? EV_NAV : EV_TEL, .timestamp=esp_timer_get_time(),
            .id=new_event_id(), .source=SRC_SIM_TOUCH, .touch_channel=channel,
            .command_seq=command_seq, .pc_send_ms=pc_send, .received_us=received_us, .received_epoch_ms=received_ms};
        if (channel == 0) ev.type=0;
        unsigned task=channel == 0 ? 3 : 2;
        if (xQueueSend(channel == 0 ? qFS : qNav, &ev, 0) == pdTRUE) return "OK\n";
        event_drop(task); return "FILA_CHEIA\n";
    }
    if (strcmp(cmd, "HELLO") == 0) return "OK\n";
    if (strcmp(cmd, "START") == 0) {
        portENTER_CRITICAL(&metric_lock);
        experiment_active = false;
        portEXIT_CRITICAL(&metric_lock);
        xQueueReset(qNav); xQueueReset(qFS); xQueueReset(qCtrl);
        for (unsigned i=0; i<7+portNUM_PROCESSORS; i++) xQueueReset(qSummary[i]);
        xSemaphoreTake(drone_lock,portMAX_DELAY);
        drone_init(&drone);
        xSemaphoreGive(drone_lock);
        portENTER_CRITICAL(&log_lock); logs_perdidos=0; portEXIT_CRITICAL(&log_lock);
        experiment_trace_reset();
        portENTER_CRITICAL(&metric_lock);
        memset(metrics, 0, sizeof(metrics));
        memset(events_expected, 0, sizeof(events_expected));
        experiment_run++;
        experiment_start=esp_timer_get_time();
        experiment_active=true;
        portEXIT_CRITICAL(&metric_lock);
        static char started[48];
        snprintf(started, sizeof(started), "OK RUN %lu\n", (unsigned long)experiment_run);
        return started;
    }
    if (strcmp(cmd, "STOP") == 0) {
        portENTER_CRITICAL(&metric_lock); experiment_active=false; portEXIT_CRITICAL(&metric_lock);
        experiment_trace_pause();
        return "OK\n";
    }
    if (strcmp(cmd, "NAV") == 0)
    {
        event_t ev = {.type = EV_NAV, .timestamp = esp_timer_get_time(), .id = new_event_id(), .source = SRC_PC, .command_seq = command_seq, .pc_send_ms = pc_send, .received_us = received_us, .received_epoch_ms = received_ms};
        if (xQueueSend(qNav, &ev, 0) == pdTRUE)
            return "OK\n";
        event_drop(2);
        return "FILA_CHEIA\n";
    }
    if (strcmp(cmd, "FS") == 0)
    {
        esp_timer_stop(led_timer);
        gpio_set_level(BLINK_GPIO, 1);
        ESP_ERROR_CHECK(esp_timer_start_once(led_timer, 1000000));
        event_t ev = {.timestamp = esp_timer_get_time(), .id = new_event_id(), .source = SRC_PC, .command_seq = command_seq, .pc_send_ms = pc_send, .received_us = received_us, .received_epoch_ms = received_ms};
        if (xQueueSend(qFS, &ev, 0) == pdTRUE)
            return "OK\n";
        event_drop(3);
        return "FILA_CHEIA\n";
    }
    return "COMANDO_INVALIDO\n";
}

// Stop-and-wait client: replay the last ACK, never re-enqueue the same command.
static const char *monitor_command(const char *cmd, int64_t received_us, int64_t received_ms)
{
    static char previous[256], response[384];
    static uint32_t last_sequence;
    static bool cached;
    uint32_t sequence;
    long long sent;
    char action[16], extra;
    if (sscanf(cmd, "%" SCNu32 " %lld %15s %c", &sequence, &sent, action, &extra) != 3)
        return "COMANDO_INVALIDO\n";
    if (cached && strcmp(previous, cmd) == 0) return response;
    if (strcmp(action, "HELLO") == 0) cached=false; // A new client session starts with HELLO.
    if (cached && sequence <= last_sequence) return "SEQUENCIA_ANTIGA\n";
    const char *result=execute_command(cmd, received_us, received_ms);
    snprintf(response, sizeof(response), "%s", result);
    snprintf(previous, sizeof(previous), "%s", cmd);
    last_sequence=sequence;
    cached=true;
    return response;
}

// Todos os sockets sao nao bloqueantes: a rede nao deve suspender o ciclo.
static int monitor_socket(int type, int port)
{
    int fd = socket(AF_INET, type, 0);
    if (fd < 0)
        return -1;
    int flags = fcntl(fd, F_GETFL, 0);
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0 ||
        bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        (type == SOCK_STREAM && listen(fd, 1) < 0))
    {
        close(fd);
        return -1;
    }
    return fd;
}

static void task_monitor(void *arg)
{
    TickType_t proximo = xTaskGetTickCount();
    int64_t release = esp_timer_get_time();
    int udp = -1, servidor = -1, cliente = -1;
    char entrada[256];
    size_t usados = 0;
    char tcp_saida[2048];
    char tcp_ack[768];
    size_t tcp_ack_total=0, tcp_ack_sent=0;
    size_t tcp_total = 0, tcp_enviado = 0;
    int64_t udp_warning_at = -5000000;
    bool udp_peer_known = false;
    char udp_ack[512];
    size_t udp_ack_len=0;
    struct sockaddr_in udp_ack_peer;
    struct sockaddr_in destino = {.sin_family = AF_INET, .sin_port = htons(PC_PORT)};
    inet_pton(AF_INET, PC_IP, &destino.sin_addr);

    for (;;)
    {
        int64_t inicio = esp_timer_get_time();
        experiment_trace_begin(4);
        // Ao perder o IP, fecha sockets e recria somente apos reconectar.
        if (!(xEventGroupGetBits(wifi_events) & WIFI_CONNECTED_BIT))
        {
            if (cliente >= 0)
                close(cliente);
            if (servidor >= 0)
                close(servidor);
            if (udp >= 0)
                close(udp);
            cliente = servidor = udp = -1;
            usados = 0;
            udp_peer_known = false;
            udp_ack_len=0;
            tcp_total = tcp_enviado = 0;
            tcp_ack_total=tcp_ack_sent=0;
        }
        else
        {
            if (!CONFIG_DRONE_PROTOCOL_TCP && udp < 0)
                udp = monitor_socket(SOCK_DGRAM, PC_PORT);
            if (CONFIG_DRONE_PROTOCOL_TCP && servidor < 0)
                servidor = monitor_socket(SOCK_STREAM, TCP_PORT);
            if (servidor >= 0 && cliente < 0)
            {
                cliente = accept(servidor, NULL, NULL);
                if (cliente >= 0)
                {
                    tcp_ack_total=tcp_ack_sent=0;
                    tcp_total=tcp_enviado=0;
                    int flags = fcntl(cliente, F_GETFL, 0);
                    int no_delay=1;
                    if (flags < 0 || fcntl(cliente, F_SETFL, flags | O_NONBLOCK) < 0 ||
                        setsockopt(cliente,IPPROTO_TCP,TCP_NODELAY,&no_delay,sizeof(no_delay)) < 0)
                    {
                        close(cliente);
                        cliente = -1;
                    }
                }
            }
            // Um datagrama representa um comando; responde ao remetente.
            if (udp >= 0)
            {
                struct sockaddr_in origem;
                socklen_t tamanho = sizeof(origem);
                char datagrama[256];
                int n = udp_ack_len ? -1 : recvfrom(udp, datagrama, sizeof(datagrama) - 1, 0,
                                 (struct sockaddr *)&origem, &tamanho);
                if (n > 0)
                {
                    int64_t rx_us = esp_timer_get_time(), rx_ms = epoch_ms();
                    datagrama[n] = 0;
                    datagrama[strcspn(datagrama, "\r\n")] = 0;
                    const char *resposta = monitor_command(datagrama, rx_us, rx_ms);
                    if (strncmp(resposta, "OK", 2) == 0) { destino = origem; udp_peer_known = true; }
                    int tam = snprintf(udp_ack, sizeof(udp_ack), "ACK %s %lld %s", datagrama, (long long)epoch_ms(), resposta);
                    if (tam > 0 && tam < sizeof(udp_ack)) {
                        udp_ack_len=tam;
                        udp_ack_peer=origem;
                    }
                }
            }
            // TCP e um fluxo: acumula fragmentos ate encontrar uma quebra de linha.
            if (cliente >= 0)
            {
                char buf[128];
                int n = tcp_ack_total == tcp_ack_sent ? recv(cliente, buf, sizeof(buf), 0) : -1;
                if (tcp_ack_total != tcp_ack_sent)
                    errno = EAGAIN;
                if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    close(cliente);
                    cliente = -1;
                    usados = 0;
                }
                else if (n > 0)
                {
                    int64_t rx_us = esp_timer_get_time(), rx_ms = epoch_ms();
                    for (int i = 0; i < n; i++)
                    {
                        if (buf[i] == '\n')
                        {
                            entrada[usados] = 0;
                            entrada[strcspn(entrada, "\r")] = 0;
                            const char *resposta = monitor_command(entrada, rx_us, rx_ms);
                            // Uma resposta pendente preserva bytes em envios parciais.
                            int tam = snprintf(tcp_ack, sizeof(tcp_ack), "ACK %s %lld %s", entrada, (long long)epoch_ms(), resposta);
                            if (tam > 0 && tam < sizeof(tcp_ack))
                            {
                                tcp_ack_total = tam;
                                tcp_ack_sent = 0;
                            }
                            else
                            {
                                close(cliente);
                                cliente = -1;
                                break;
                            }
                            usados = 0;
                            // Um comando por leitura; feche se vierem linhas adicionais para nao perder dados.
                            if (i + 1 < n)
                            {
                                close(cliente);
                                cliente = -1;
                                tcp_total = tcp_enviado = 0;
                                break;
                            }
                        }
                        else if (usados < sizeof(entrada) - 1)
                            entrada[usados++] = buf[i];
                        else
                        {
                            usados = 0;
                            close(cliente);
                            cliente = -1;
                            break;
                        }
                    }
                }
            }
            // Envia registros individuais; limita o trabalho por ciclo e contabiliza perdas da fila.
            for (int i = 0; i < (CONFIG_DRONE_PROTOCOL_TCP ? 6 : 1); i++)
            {
                if (udp >= 0 && udp_ack_len) {
                    if (sendto(udp, udp_ack, udp_ack_len, 0, (struct sockaddr *)&udp_ack_peer, sizeof(udp_ack_peer)) < 0)
                        break; // ACKs have priority and retain their bytes until accepted by the stack.
                    udp_ack_len=0;
                }
                if (udp >= 0 && !udp_peer_known) break;
                if (udp >= 0 && tcp_total) {
                    stamp_send(tcp_saida);
                    int sent = sendto(udp, tcp_saida, tcp_total, 0, (struct sockaddr *)&destino, sizeof(destino));
                    if (sent < 0) {
                        int error = errno;
                        int64_t now = esp_timer_get_time();
                        if (now - udp_warning_at >= 5000000) {
                            ESP_LOGW(TAG, "UDP pendente: errno=%d heap=%u; verifique IP/rede do PC", error,
                                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
                            udp_warning_at = now;
                        }
                        break; // Keep bytes and retry next cycle, without draining more records.
                    }
                    tcp_total = tcp_enviado = 0;
                }
                if (CONFIG_DRONE_PROTOCOL_TCP && cliente < 0)
                    break;
                if (cliente >= 0 && tcp_enviado < tcp_total)
                {
                    if (!tcp_enviado) stamp_send(tcp_saida);
                    int sent = send(cliente, tcp_saida + tcp_enviado, tcp_total - tcp_enviado, 0);
                    if (sent > 0)
                        tcp_enviado += sent;
                    else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
                    {
                        close(cliente);
                        cliente = -1;
                        tcp_total = tcp_enviado = 0;
                    }
                    if (cliente >= 0 && tcp_enviado < tcp_total)
                        break;
                }
                if (cliente >= 0 && tcp_ack_sent < tcp_ack_total) {
                    int sent=send(cliente,tcp_ack+tcp_ack_sent,tcp_ack_total-tcp_ack_sent,0);
                    if (sent > 0) tcp_ack_sent+=sent;
                    else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                        close(cliente); cliente=-1;
                        tcp_total=tcp_enviado=tcp_ack_total=tcp_ack_sent=0;
                        break;
                    }
                    if (tcp_ack_sent < tcp_ack_total) break;
                }
                summary_t summary;
                bool has_summary = false;
                for (unsigned slot = 0; slot < 7 + portNUM_PROCESSORS; slot++)
                    if (xQueueReceive(qSummary[slot], &summary, 0) == pdTRUE) {
                        has_summary = true;
                        break;
                    }
                if (has_summary)
                {
                    size_t len = packet_size(summary.text, sizeof(summary.text));
                    if (udp >= 0 || cliente >= 0)
                    {
                        memcpy(tcp_saida, summary.text, len);
                        tcp_total = len;
                        tcp_enviado = 0;
                    }
                    continue;
                }
                break; // Only summaries leave the ESP; no per-execution telemetry queue.
            }
            // Reenvia apenas os bytes que faltam em caso de envio parcial.
            if (cliente >= 0 && tcp_enviado < tcp_total)
            {
                if (!tcp_enviado) stamp_send(tcp_saida);
                int n = send(cliente, tcp_saida + tcp_enviado, tcp_total - tcp_enviado, 0);
                if (n > 0)
                    tcp_enviado += n;
                else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    ESP_LOGW(TAG, "Conexao TCP encerrada: %d", errno);
                    close(cliente);
                    cliente = -1;
                    tcp_total = tcp_enviado = usados = 0;
                }
            }
        }
        // Mede a duracao do ciclo e verifica seu prazo de 500 ms.
        int64_t fim = esp_timer_get_time();
        registrar_evento(4, (event_t){.timestamp = release, .id = new_event_id(), .source = SRC_PERIODIC}, inicio, fim);
        if (fim - inicio > 500000)
            ESP_LOGW(TAG, "MONITOR excedeu 500 ms");
        if (xTaskDelayUntil(&proximo, pdMS_TO_TICKS(10)) == pdFALSE)
            vTaskDelay(1); // An overdue cycle must still let lower-priority tasks run.
        release += 10000;
    }
}

// Apenas para Log
static void time_sync_notification_cb(struct timeval *tv)
{
    portENTER_CRITICAL(&tempo_lock);
    hora_sincronizada = true;
    ciclos_sntp = 0;
    portEXIT_CRITICAL(&tempo_lock);
    ESP_LOGI(TAG, "Tempo sincronizado via SNTP.");
}
// #
// TASK - 6 TIME_TASK
// D - 1000ms (1s)
// Captura a hora de um servidor  SNTP e monitora quantos ciclos se passaram desde a atualizacao do servidor
// #
static void task_time(void *arg)
{
    // Aguarda o primeiro IP antes de iniciar o SNTP.
    xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    // Define fuso (Brasil sem DST): BRT-3
    setenv("TZ", "BRT-3", 1);
    tzset();

    // SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_setservername(0, "a.st1.ntp.br");
    esp_sntp_setservername(1, "b.st1.ntp.br");
    esp_sntp_setservername(2, "pool.ntp.org");
    esp_sntp_init();

    // Periodicidade de um segundo; o callback reinicia o contador de ciclos.
    TickType_t proximo = xTaskGetTickCount();
    int64_t release = esp_timer_get_time();
    for (;;)
    {
        int64_t inicio = esp_timer_get_time();
        experiment_trace_begin(5);
        portENTER_CRITICAL(&tempo_lock);
        if (hora_sincronizada)
            ciclos_sntp++;
        portEXIT_CRITICAL(&tempo_lock);
        int64_t fim = esp_timer_get_time();
        registrar_evento(5, (event_t){.timestamp = release, .id = new_event_id(), .source = SRC_PERIODIC}, inicio, fim);
        if (xTaskDelayUntil(&proximo, pdMS_TO_TICKS(1000)) == pdFALSE)
            vTaskDelay(1); // An overdue cycle must still let lower-priority tasks run.
        release += 1000000;
    }
}
// #

// ======================================================
// TOUCH SENSOR
// ESP-IDF 6.1
// ======================================================

#define EXAMPLE_TOUCH_SAMPLE_CFG_NUM 1
#define EXAMPLE_TOUCH_CHANNEL_NUM 4
#define EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES 32
static touch_channel_handle_t touch_channels[EXAMPLE_TOUCH_CHANNEL_NUM];

// ======================================================
// CANAIS TOUCH
//
// CH4 -> GPIO13 -> NAV
// CH9 -> GPIO32 -> TELEMETRIA
// CH0 -> GPIO4  -> FAIL-SAFE
// ======================================================

static int s_channel_id[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    4,
    9,
    0,
    3};

// Minimum relative change from the untouched calibration baseline.
static float s_thresh2bm_ratio[EXAMPLE_TOUCH_CHANNEL_NUM] = {

    [0 ... EXAMPLE_TOUCH_CHANNEL_NUM - 1] = CONFIG_DRONE_TOUCH_THRESHOLD_PERCENT / 100.0f};

// ======================================================
// CALLBACK - TOUCH ATIVADO
// ======================================================

static bool example_touch_on_active_cb(
    touch_sensor_handle_t sens_handle,
    const touch_active_event_data_t *event,
    void *user_ctx)
{
    portENTER_CRITICAL_SAFE(&event_lock);
    uint32_t bit=1U << event->chan_id;
    bool accept=physical_touch_enabled && !(physical_touch_seen & bit);
    if (accept) physical_touch_seen |= bit;
    portEXIT_CRITICAL_SAFE(&event_lock);
    if (!accept) return false;
    BaseType_t higher_priority_task_woken = pdFALSE;
    switch (event->chan_id)
    {

    case 4:
    {
        event_t ev = {.type = EV_NAV, .timestamp = esp_timer_get_time(), .id = new_event_id(), .source = SRC_TOUCH, .touch_channel=event->chan_id};
        touches_nav++;

        if (xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken) != pdTRUE)
            event_drop(2);

        break;
    }

    case 3: // Fourth electrode: telemetry, measured separately by channel.
    case 9:
    {
        event_t ev = {.type = EV_TEL, .timestamp = esp_timer_get_time(), .id = new_event_id(), .source = SRC_TOUCH, .touch_channel=event->chan_id};
        touches_tel++;

        if (xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken) != pdTRUE)
            event_drop(2);

        break;
    }

    case 0:
    {
        touches_fs++;
        event_t ev = {.timestamp = esp_timer_get_time(), .id = new_event_id(), .source = SRC_TOUCH, .touch_channel=event->chan_id};
        if (xQueueSendFromISR(qFS, &ev, &higher_priority_task_woken) != pdTRUE)
            event_drop(3);

        break;
    }

    default:

        break;
    }

    return higher_priority_task_woken == pdTRUE;
}

// ======================================================
// CALLBACK - TOUCH LIBERADO
// ======================================================

static bool example_touch_on_inactive_cb(
    touch_sensor_handle_t sens_handle,
    const touch_inactive_event_data_t *event,
    void *user_ctx)
{
    return false;
}

// ======================================================
// CALIBRAÇÃO INICIAL DOS TOUCHES
// ======================================================

static void example_touch_do_initial_scanning(
    touch_sensor_handle_t sens_handle,
    touch_channel_handle_t chan_handle[])
{
    // Liga o sensor

    ESP_ERROR_CHECK(
        touch_sensor_enable(
            sens_handle));

    // Faz algumas leituras iniciais para estabilizar

    for (
        int i = 0;
        i < EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES;
        i++)
    {

        ESP_ERROR_CHECK(
            touch_sensor_trigger_oneshot_scanning(
                sens_handle,
                2000));
    }

    // Desliga temporariamente

    ESP_ERROR_CHECK(
        touch_sensor_disable(
            sens_handle));

    printf(
        "Initial benchmark and new threshold are:\n");

    // ==================================================
    // Obtém benchmark de cada canal
    // ==================================================

    for (
        int i = 0;
        i < EXAMPLE_TOUCH_CHANNEL_NUM;
        i++)
    {

        uint32_t benchmark[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};

#if SOC_TOUCH_SUPPORT_BENCHMARK

        ESP_ERROR_CHECK(
            touch_channel_read_data(
                chan_handle[i],
                TOUCH_CHAN_DATA_TYPE_BENCHMARK,
                benchmark));

#else

        ESP_ERROR_CHECK(
            touch_channel_read_data(
                chan_handle[i],
                TOUCH_CHAN_DATA_TYPE_SMOOTH,
                benchmark));

#endif

        printf(
            "Touch [CH %d]",
            s_channel_id[i]);

        // Pega configuração padrão

        touch_channel_config_t chan_cfg =
            EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();

        // ==================================================
        // Calcula threshold
        // ==================================================

        for (
            int j = 0;
            j < EXAMPLE_TOUCH_SAMPLE_CFG_NUM;
            j++)
        {

#if SOC_TOUCH_SENSOR_VERSION == 1

            // ESP32 clássico -> Touch V1
            // O valor diminui ao tocar

            chan_cfg.abs_active_thresh[j] =
                (uint32_t)(benchmark[j] *
                           (1 - s_thresh2bm_ratio[i]));

            printf(
                " %d: %" PRIu32 ", %" PRIu32 "\t",
                j,
                benchmark[j],
                chan_cfg.abs_active_thresh[j]);

#else

            chan_cfg.active_thresh[j] =
                (uint32_t)(benchmark[j] *
                           s_thresh2bm_ratio[i]);

            printf(
                " %d: %" PRIu32 ", %" PRIu32 "\t",
                j,
                benchmark[j],
                chan_cfg.active_thresh[j]);

#endif
        }

        printf("\n");

        // Atualiza configuração do canal

        ESP_ERROR_CHECK(
            touch_sensor_reconfig_channel(
                chan_handle[i],
                &chan_cfg));
    }
}

// ======================================================
// INICIALIZAÇÃO DO TOUCH
// ======================================================

static esp_err_t example_touch_init(void)
{
    touch_sensor_handle_t sens_handle = NULL;

    touch_channel_handle_t *chan_handle = touch_channels;

    // ==================================================
    // 1. Cria controller
    // ==================================================

    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = {
        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(1.0, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_1V7)
    };

    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(
            EXAMPLE_TOUCH_SAMPLE_CFG_NUM,
            sample_cfg);

    ESP_ERROR_CHECK(
        touch_sensor_new_controller(
            &sens_cfg,
            &sens_handle));

    // ==================================================
    // 2. Cria os três canais
    // ==================================================

    touch_channel_config_t chan_cfg =
        EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();

    for (
        int i = 0;
        i < EXAMPLE_TOUCH_CHANNEL_NUM;
        i++)
    {

        ESP_ERROR_CHECK(
            touch_sensor_new_channel(
                sens_handle,
                s_channel_id[i],
                &chan_cfg,
                &chan_handle[i]));

        touch_chan_info_t chan_info = {};

        ESP_ERROR_CHECK(
            touch_sensor_get_channel_info(
                chan_handle[i],
                &chan_info));

        printf(
            "Touch [CH %d] enabled on GPIO%d\n",
            s_channel_id[i],
            chan_info.chan_gpio);
    }

    // ==================================================
    // 3. Configura filtro
    // ==================================================

    touch_sensor_filter_config_t filter_cfg =
        TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    filter_cfg.interval_ms = 50;

    ESP_ERROR_CHECK(
        touch_sensor_config_filter(
            sens_handle,
            &filter_cfg));

    // ==================================================
    // 4. Calibração inicial
    // ==================================================

    example_touch_do_initial_scanning(
        sens_handle,
        chan_handle);

    // ==================================================
    // 5. Registra callbacks
    // ==================================================

    touch_event_callbacks_t callbacks = {

        .on_active =
            example_touch_on_active_cb,

        .on_inactive =
            example_touch_on_inactive_cb,
    };

    ESP_ERROR_CHECK(
        touch_sensor_register_callbacks(
            sens_handle,
            &callbacks,
            NULL));

    // ==================================================
    // 6. Liga o sensor
    // ==================================================

    ESP_ERROR_CHECK(
        touch_sensor_enable(
            sens_handle));

    physical_sensor = sens_handle;
    // Scan only during the physical test; simulated events do not need the filter timer.

    ESP_LOGI(
        TAG,
        "Touch sensor pronto");

    return ESP_OK;
}

// Resumo cumulativo: nenhuma impressao ocorre dentro de uma secao critica.
static void print_metrics(bool verbose)
{
    // Only app_main publishes summaries; reuse one buffer instead of stacking three.
    static summary_t summary;
    if (verbose) printf("STACK main minimum_free_bytes=%u\n", (unsigned)uxTaskGetStackHighWaterMark(NULL));
    portENTER_CRITICAL(&tempo_lock);
    bool synced=hora_sincronizada;
    portEXIT_CRITICAL(&tempo_lock);
    for (unsigned i=0; i<EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        uint32_t value[EXAMPLE_TOUCH_SAMPLE_CFG_NUM];
        if (touch_channels[i] && touch_channel_read_data(touch_channels[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, value) == ESP_OK)
            if (verbose) printf("TOUCH_SAMPLE channel=%d smooth=%lu threshold_percent=%d\n", s_channel_id[i],
                   (unsigned long)value[0], CONFIG_DRONE_TOUCH_THRESHOLD_PERCENT);
    }
    if (hMON && verbose) printf("STACK MON minimum_free_bytes=%u allocated_bytes=%u\n",
                     (unsigned)uxTaskGetStackHighWaterMark(hMON), STK_MON);
    for (unsigned i = 0; i < 6; i++)
    {
        uint32_t runs, misses, drops, windows, violations;
        uint64_t r, c, l, j;
        unsigned h;
        uint32_t run, expected, lock_samples, lower_holder, touch_mask, pc_samples;
        int64_t pc_max, last_pc, last_rx, last_rx_us;
        int64_t worst[3], last[3];
        uint64_t lock_max, lock_total, response_sum;
        bool final;
        portENTER_CRITICAL(&metric_lock);
        run=experiment_run; final=!experiment_active;
        timing_metrics_t *m = &metrics[i];
        expected=events_expected[i];
        runs = m->runs;
        misses = m->misses;
        drops = m->drops;
        windows = m->windows;
        violations = m->violations;
        r = m->response_max;
        c = m->execution_max;
        l = m->latency_max;
        j = m->jitter_max;
        h = timing_hwm99_bin(m);
        worst[0]=m->worst_event_us; worst[1]=m->worst_start_us; worst[2]=m->worst_end_us;
        last[0]=m->last_event_us; last[1]=m->last_start_us; last[2]=m->last_end_us;
        lock_max=m->lock_wait_max_us; lock_total=m->lock_wait_total_us; lock_samples=m->lock_samples; lower_holder=m->lower_priority_holder;
        response_sum=m->response_sum;
        touch_mask=m->simulated_touch_mask; pc_samples=m->pc_samples; pc_max=m->pc_activation_max_us;
        last_pc=m->last_pc_send_ms; last_rx=m->last_rx_ms; last_rx_us=m->last_rx_us;
        portEXIT_CRITICAL(&metric_lock);
        snprintf(summary.text, sizeof(summary.text), "{" EXP_META ",\"kind\":\"metric\",\"sntp\":%s,\"events_expected\":%lu,\"run\":%lu,\"final\":%s,\"task\":\"%s\",\"epoch_ms\":%lld,\"runs\":%lu,\"misses\":%lu,\"drops\":%lu,\"WCRT_us\":%llu,\"HWM99_bin_us\":%u,\"windows\":%lu,\"violations\":%lu,\"logs_lost\":%lu}\n",
                 synced ? "true" : "false", (unsigned long)expected, (unsigned long)run, final ? "true" : "false", metric_names[i], (long long)epoch_ms(), (unsigned long)runs, (unsigned long)misses, (unsigned long)drops,
                 (unsigned long long)r, h * HIST_US, (unsigned long)windows, (unsigned long)violations, (unsigned long)logs_perdidos);
        experiment_trace_sample_t trace=experiment_trace_read(metric_names[i]);
        size_t length=strlen(summary.text)-2;
        int added=snprintf(summary.text+length,sizeof(summary.text)-length,
            ",\"deadline_us\":%lu,\"C_wall_max_us\":%llu,\"latency_max_us\":%llu,\"jitter_max_us\":%llu,\"response_sum_us\":%llu,\"hwm_overflow\":%s,\"worst_timestamps_us\":[%lld,%lld,%lld],\"last_timestamps_us\":[%lld,%lld,%lld],\"lock_wait_max_us\":%llu,\"lock_wait_total_us\":%llu,\"resource\":\"drone_mutex_PI\",\"lock_samples\":%lu,\"lower_priority_holder\":%lu,\"off_cpu_total_us\":%llu,\"switches_total\":%lu,\"sim_touch_mask\":%lu,\"pc_samples\":%lu,\"pc_activation_max_us\":%lld,\"last_pc_send_ms\":%lld,\"last_rx_ms\":%lld,\"last_rx_us\":%lld}\n",
            (unsigned long)deadlines[i],(unsigned long long)c,(unsigned long long)l,(unsigned long long)j,
            (unsigned long long)response_sum,h==HIST_BINS-1 ? "true" : "false",
            (long long)worst[0],(long long)worst[1],(long long)worst[2],(long long)last[0],(long long)last[1],(long long)last[2],
            (unsigned long long)lock_max,(unsigned long long)lock_total,(unsigned long)lock_samples,(unsigned long)lower_holder,
            (unsigned long long)trace.off_cpu_us,(unsigned long)trace.switches,(unsigned long)touch_mask,(unsigned long)pc_samples,
            (long long)pc_max,(long long)last_pc,(long long)last_rx,(long long)last_rx_us);
        if (added < 0 || (size_t)added >= sizeof(summary.text)-length) ESP_LOGE(TAG,"Resumo excedeu buffer");
        else xQueueOverwrite(qSummary[i], &summary);
        if (verbose) printf("METRIC %s runs=%lu misses=%lu drops=%lu WCRT_us=%llu C_wall_max_us=%llu latency_max_us=%llu jitter_max_us=%llu HWM99_bin_us=%u overflow=%u mk=%d/%d windows=%lu violations=%lu\n",
               metric_names[i], (unsigned long)runs, (unsigned long)misses, (unsigned long)drops,
               (unsigned long long)r, (unsigned long long)c, (unsigned long long)l, (unsigned long long)j,
               h * HIST_US, h == HIST_BINS - 1, MK_M, MK_K, (unsigned long)windows, (unsigned long)violations);
    }
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    static TaskStatus_t tasks[32];
    static uint32_t last_total, last_idle[portNUM_PROCESSORS];
    uint32_t total = 0, idle[portNUM_PROCESSORS] = {0};
    UBaseType_t count = uxTaskGetSystemState(tasks, 32, &total);
    if (count)
    {
        for (unsigned i = 0; i < count; i++)
        {
            for (unsigned core = 0; core < portNUM_PROCESSORS; core++)
                if (tasks[i].xHandle == xTaskGetIdleTaskHandleForCore(core))
                    idle[core] = tasks[i].ulRunTimeCounter;
        }
        uint32_t elapsed = total - last_total;
        for (unsigned core = 0; core < portNUM_PROCESSORS; core++)
        {
            float busy = elapsed ? 100.0f * (1.0f - (float)(idle[core] - last_idle[core]) / elapsed) : 0;
            if (verbose) printf("CPU core=%u busy_pct=%.2f\n", core, limit(busy, 0, 100));
                snprintf(summary.text, sizeof(summary.text), "{" EXP_META ",\"kind\":\"cpu\",\"run\":%lu,\"core\":%u,\"busy_pct\":%.2f,\"interval_us\":%lu,\"epoch_ms\":%lld}\n", (unsigned long)experiment_run, core, limit(busy, 0, 100), (unsigned long)elapsed, (long long)epoch_ms());
            xQueueOverwrite(qSummary[6 + core], &summary);
            last_idle[core] = idle[core];
        }
        last_total = total;
    }
    else
        ESP_LOGW(TAG, "Aumente a capacidade do snapshot de tarefas para medir CPU");
#endif
    xSemaphoreTake(drone_lock,portMAX_DELAY);
    drone_t snapshot = drone;
    xSemaphoreGive(drone_lock);
    snprintf(summary.text,sizeof(summary.text),"{" EXP_META ",\"kind\":\"state\",\"run\":%lu,\"epoch_ms\":%lld,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"altitude\":%.2f,\"x\":%.2f,\"y\":%.2f,\"waypoint\":%u,\"emergency\":%s}\n",
        (unsigned long)experiment_run,(long long)epoch_ms(),snapshot.estimate.roll,snapshot.estimate.pitch,
        snapshot.estimate.yaw,snapshot.estimate.altitude,snapshot.estimate.x,snapshot.estimate.y,
        snapshot.waypoint,snapshot.emergency ? "true" : "false");
    xQueueOverwrite(qSummary[6+portNUM_PROCESSORS],&summary);
    if (verbose) printf("STATE roll=%.2f pitch=%.2f yaw=%.2f altitude=%.2f x=%.2f y=%.2f waypoint=%u emergency=%u motors=%.1f,%.1f,%.1f,%.1f\n",
           snapshot.estimate.roll, snapshot.estimate.pitch, snapshot.estimate.yaw, snapshot.estimate.altitude,
           snapshot.estimate.x, snapshot.estimate.y, snapshot.waypoint, snapshot.emergency,
           snapshot.motors.m1, snapshot.motors.m2, snapshot.motors.m3, snapshot.motors.m4);
}

// ======================================================
// APP MAIN
// ======================================================

void app_main(void)
{
    drone_lock=xSemaphoreCreateMutex();
    assert(drone_lock);
    ESP_ERROR_CHECK(nvs_flash_init());
    // 1. Reseta o pino para o estado padrão
    gpio_reset_pin(BLINK_GPIO);

    // 2. Configura o pino como Saída (Output)
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    const esp_timer_create_args_t led_timer_args = {
        .callback = apagar_led,
        .name = "led_fs",
    };

    ESP_ERROR_CHECK(esp_timer_create(&led_timer_args, &led_timer));

    // inicializar wifi
    wifi_init();
    // IPC
    drone_init(&drone);
    // Keep the latest snapshot per task/core while the network is unavailable.
    for (unsigned i = 0; i < 7 + portNUM_PROCESSORS; i++) {
        qSummary[i] = xQueueCreate(1, sizeof(summary_t));
        assert(qSummary[i]);
    }
    qNav = xQueueCreate(16, sizeof(event_t));
    qFS = xQueueCreate(16, sizeof(event_t));
    qCtrl = xQueueCreate(16, sizeof(event_t));
    assert(qNav && qFS && qCtrl);

    // Cria tasks

    ESP_ERROR_CHECK(xTaskCreate(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_monitor, "MON", STK_MON, NULL, PRIO_MON, &hMON) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_time, "TIME", STK, NULL, PRIO_TIME, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(example_touch_init());

    ESP_LOGI(TAG, "Sistema iniciado: heap interno livre=%u maior_bloco=%u resumos=500ms",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    while (1)
    {

        vTaskDelay(
            pdMS_TO_TICKS(500));
        static unsigned snapshots;
        print_metrics(++snapshots % 10 == 0);
    }
}
