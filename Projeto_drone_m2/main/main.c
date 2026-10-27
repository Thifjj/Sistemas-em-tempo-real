/*
 * Drone ESP32 + FreeRTOS
 * ESP-IDF 6.1
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
#include "esp_log.h"
#include "esp_timer.h"
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

#define TAG "DRONE"

// ======================================================
// PERIODICIDADE E PRIORIDADES
// ======================================================

#define FUS_T_MS 5

#if CONFIG_FREERTOS_HZ < 200
#error "FUS_IMU requer CONFIG_FREERTOS_HZ >= 200 para um periodo de 5 ms"
#endif

#define PRIO_MON 5
#define PRIO_TIME 2
#define PRIO_FS_TASK 5
#define PRIO_FUS_IMU 4
#define PRIO_CTRL_ATT 3
#define PRIO_NAV_PLAN 2
#define BLINK_GPIO 2
#define STK 6144

#define D_FUS_IMU 5

#define D_CTRL_ATT 5

#define D_NAV_PLAN 20

#define D_FS_TASK 10

// Cada evento transporta timestamp, sequencia local e origem.
typedef enum { SRC_TOUCH, SRC_PC, SRC_FUSION, SRC_PERIODIC } event_source_t;
typedef struct { int type; int64_t timestamp; uint32_t id; event_source_t source; state_t sample; } event_t;
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
static portMUX_TYPE drone_lock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t qCtrl, qFS;
static const char *metric_names[] = {"FUS_IMU","CTRL_ATT","NAV_PLAN","FS_TASK","MONITOR_TASK","TIME_TASK"};
static const uint32_t deadlines[] = {5000,5000,20000,10000,500000,1000000};
static timing_metrics_t metrics[6];
static portMUX_TYPE metric_lock = portMUX_INITIALIZER_UNLOCKED;
static void event_drop(unsigned task)
{
    portENTER_CRITICAL_SAFE(&metric_lock); metrics[task].drops++; portEXIT_CRITICAL_SAFE(&metric_lock);
}
// ======================================================
// HANDLES DAS TASKS
// ======================================================

static TaskHandle_t hFUS = NULL;
static TaskHandle_t hCTRL = NULL;
static TaskHandle_t hNAV = NULL;
static TaskHandle_t hFS = NULL;

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


// Fila limitada: tarefas nunca esperam pela rede; perdas ficam contabilizadas.
typedef struct { const char *task; int64_t evento, inicio, fim; uint32_t id; event_source_t source; } log_evento_t;
static QueueHandle_t qLog;
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
    timing_add(&metrics[task], event.timestamp, inicio, fim, deadlines[task], task == 2);
    portEXIT_CRITICAL(&metric_lock);
    log_evento_t item = {metric_names[task], event.timestamp, inicio, fim, event.id, event.source};
    if (xQueueSend(qLog, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&log_lock); logs_perdidos++; portEXIT_CRITICAL(&log_lock);
    }
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
    for (;;) {
        int64_t start = esp_timer_get_time();
        float dt = limit((start-previous)/1000000.0f, .001f, .05f); previous=start;
        event_t event = {.timestamp=release,.id=new_event_id(),.source=SRC_PERIODIC};
        portENTER_CRITICAL(&drone_lock);
        drone_fuse(&drone, dt);
        state_t sample = drone.estimate;
        portEXIT_CRITICAL(&drone_lock);
        cpu_tight_loop_us(1000);
        event_t control = {.timestamp=esp_timer_get_time(), .id=event.id,
                           .source=SRC_FUSION,.sample=sample};
        if(xQueueSend(qCtrl,&control,0)!=pdTRUE) event_drop(1);
        registrar_evento(0,event,start,esp_timer_get_time());
        vTaskDelayUntil(&next,pdMS_TO_TICKS(FUS_T_MS));
        release += FUS_T_MS*1000LL;
    }
}

// PID de atitude e altitude usa o estado da amostra que originou o evento.
static void task_ctrl_att(void *arg)
{
    event_t event;
    int64_t previous = 0;
    for (;;) if(xQueueReceive(qCtrl,&event,portMAX_DELAY)==pdTRUE) {
        int64_t start=esp_timer_get_time();
        float dt=previous ? limit((event.timestamp-previous)/1000000.0f,.001f,.05f) : .005f;
        previous=event.timestamp;
        portENTER_CRITICAL(&drone_lock);
        drone_control(&drone,event.sample,dt);
        portEXIT_CRITICAL(&drone_lock);
        cpu_tight_loop_us(800);
        registrar_evento(1,event,start,esp_timer_get_time());
    }
}

static void task_nav_plan(void *arg)
{
    event_t event;
    for (;;) if(xQueueReceive(qNav,&event,portMAX_DELAY)==pdTRUE) {
        int64_t start=esp_timer_get_time();
        portENTER_CRITICAL(&drone_lock);
        if(event.type==EV_NAV) drone_navigate(&drone);
        state_t state=drone.estimate;
        portEXIT_CRITICAL(&drone_lock);
        if(event.type==EV_TEL) printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f altitude=%.2f\n",
                                    state.roll,state.pitch,state.yaw,state.altitude);
        cpu_tight_loop_us(event.type==EV_NAV ? 3500 : 500);
        registrar_evento(2,event,start,esp_timer_get_time());
    }
}

static void task_fail_safe(void *arg)
{
    event_t event;
    for (;;) if(xQueueReceive(qFS,&event,portMAX_DELAY)==pdTRUE) {
        int64_t start=esp_timer_get_time();
        portENTER_CRITICAL(&drone_lock);
        drone_fail_safe(&drone);
        portEXIT_CRITICAL(&drone_lock);
        cpu_tight_loop_us(900);
        registrar_evento(3,event,start,esp_timer_get_time());
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
static const char *monitor_command(const char *cmd)
{
    const char *espaco = strchr(cmd, ' ');
    if (espaco) cmd = espaco + 1;
    if (strcmp(cmd, "NAV") == 0) {
        event_t ev = {.type=EV_NAV,.timestamp=esp_timer_get_time(),.id=new_event_id(),.source=SRC_PC};
        if(xQueueSend(qNav,&ev,0)==pdTRUE) return "OK\n";
        event_drop(2); return "FILA_CHEIA\n";
    }
    if (strcmp(cmd, "FS") == 0) {
        esp_timer_stop(led_timer);
        gpio_set_level(BLINK_GPIO, 1);
        ESP_ERROR_CHECK(esp_timer_start_once(led_timer, 1000000));
        event_t ev = {.timestamp=esp_timer_get_time(),.id=new_event_id(),.source=SRC_PC};
        if(xQueueSend(qFS,&ev,0)==pdTRUE) return "OK\n";
        event_drop(3); return "FILA_CHEIA\n";
    }
    return "COMANDO_INVALIDO\n";
}

// Todos os sockets sao nao bloqueantes: a rede nao deve suspender o ciclo.
static int monitor_socket(int type, int port)
{
    int fd = socket(AF_INET, type, 0);
    if (fd < 0) return -1;
    int flags = fcntl(fd, F_GETFL, 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(port),
                                .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0 ||
        bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        (type == SOCK_STREAM && listen(fd, 1) < 0)) {
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
    uint32_t sequencia = 0;
    char tcp_saida[512];
    size_t tcp_total = 0, tcp_enviado = 0;
    struct sockaddr_in destino = { .sin_family = AF_INET, .sin_port = htons(PC_PORT) };
    inet_pton(AF_INET, PC_IP, &destino.sin_addr);

    for (;;) {
        int64_t inicio = esp_timer_get_time();
        // Ao perder o IP, fecha sockets e recria somente apos reconectar.
        if (!(xEventGroupGetBits(wifi_events) & WIFI_CONNECTED_BIT)) {
            if (cliente >= 0) close(cliente);
            if (servidor >= 0) close(servidor);
            if (udp >= 0) close(udp);
            cliente = servidor = udp = -1;
            usados = 0; tcp_total = tcp_enviado = 0;
        } else {
            if (udp < 0) udp = monitor_socket(SOCK_DGRAM, PC_PORT);
            if (servidor < 0) servidor = monitor_socket(SOCK_STREAM, TCP_PORT);
            if (servidor >= 0 && cliente < 0) {
                cliente = accept(servidor, NULL, NULL);
                if (cliente >= 0) {
                    int flags = fcntl(cliente, F_GETFL, 0);
                    if (flags < 0 || fcntl(cliente, F_SETFL, flags | O_NONBLOCK) < 0) {
                        close(cliente); cliente = -1;
                    }
                }
            }
            // Um datagrama representa um comando; responde ao remetente.
            if (udp >= 0) {
                struct sockaddr_in origem;
                socklen_t tamanho = sizeof(origem);
                char datagrama[256];
                int n = recvfrom(udp, datagrama, sizeof(datagrama) - 1, 0,
                                 (struct sockaddr *)&origem, &tamanho);
                if (n > 0) {
                    datagrama[n] = 0;
                    datagrama[strcspn(datagrama, "\r\n")] = 0;
                    const char *resposta = monitor_command(datagrama);
                    char ack[320];
                    int tam = snprintf(ack, sizeof(ack), "ACK %s %s", datagrama, resposta);
                    if (tam > 0 && tam < sizeof(ack) && sendto(udp, ack, tam, 0, (struct sockaddr *)&origem, tamanho) < 0)
                        ESP_LOGW(TAG, "Falha no envio UDP: %d", errno);
                }
            }
            // TCP e um fluxo: acumula fragmentos ate encontrar uma quebra de linha.
            if (cliente >= 0) {
                char buf[128];
                int n = tcp_total == tcp_enviado ? recv(cliente, buf, sizeof(buf), 0) : -1;
                if (tcp_total != tcp_enviado) errno = EAGAIN;
                if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                    close(cliente); cliente = -1; usados = 0;
                } else if (n > 0) {
                    for (int i = 0; i < n; i++) {
                        if (buf[i] == '\n') {
                            entrada[usados] = 0;
                            entrada[strcspn(entrada, "\r")] = 0;
                            const char *resposta = monitor_command(entrada);
                            // Uma resposta pendente preserva bytes em envios parciais.
                            int tam = snprintf(tcp_saida, sizeof(tcp_saida), "ACK %s %s", entrada, resposta);
                            if (tam > 0 && tam < sizeof(tcp_saida)) { tcp_total = tam; tcp_enviado = 0; }
                            else { close(cliente); cliente = -1; break; }
                            usados = 0;
                            // Um comando por leitura; feche se vierem linhas adicionais para nao perder dados.
                            if (i + 1 < n) { close(cliente); cliente = -1; tcp_total = tcp_enviado = 0; break; }
                        } else if (usados < sizeof(entrada) - 1) entrada[usados++] = buf[i];
                        else { usados = 0; close(cliente); cliente = -1; break; }
                    }
                }
            }
            // Envia registros individuais; limita o trabalho por ciclo e contabiliza perdas da fila.
            for (int i = 0; i < 256; i++) {
                if (cliente >= 0 && tcp_enviado < tcp_total) {
                    int sent=send(cliente,tcp_saida+tcp_enviado,tcp_total-tcp_enviado,0);
                    if(sent>0) tcp_enviado+=sent;
                    else if(sent==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK)) {
                        close(cliente); cliente=-1; tcp_total=tcp_enviado=0;
                    }
                    if(cliente>=0 && tcp_enviado<tcp_total) break;
                }
                log_evento_t item;
                if (xQueueReceive(qLog, &item, 0) != pdTRUE) break;
                struct timeval agora; gettimeofday(&agora, NULL);
                portENTER_CRITICAL(&tempo_lock);
                bool sincronizado = hora_sincronizada; uint32_t ciclos = ciclos_sntp;
                portEXIT_CRITICAL(&tempo_lock);
                portENTER_CRITICAL(&log_lock); uint32_t perdas = logs_perdidos; portEXIT_CRITICAL(&log_lock);
                char log[512];
                int n = snprintf(log, sizeof(log),
                    "{\"seq\":%" PRIu32 ",\"epoch_ms\":%lld,\"sntp\":%s,\"ciclos_sntp\":%" PRIu32 ",\"task\":\"%s\",\"event_id\":%" PRIu32 ",\"source\":%u,\"evento_us\":%lld,\"inicio_us\":%lld,\"fim_us\":%lld,\"perdidos\":%" PRIu32 "}\n",
                    sequencia++, (long long)agora.tv_sec * 1000 + agora.tv_usec / 1000,
                    sincronizado ? "true" : "false", ciclos, item.task, item.id, (unsigned)item.source,
                    (long long)item.evento, (long long)item.inicio, (long long)item.fim, perdas);
                if (n <= 0 || n >= sizeof(log)) continue;
                if (udp >= 0 && sendto(udp, log, n, 0, (struct sockaddr *)&destino, sizeof(destino)) < 0)
                    ESP_LOGW(TAG, "Falha no envio UDP: %d", errno);
                if (cliente >= 0) { memcpy(tcp_saida, log, n); tcp_total = n; tcp_enviado = 0; }
            }
            // Reenvia apenas os bytes que faltam em caso de envio parcial.
            if (cliente >= 0 && tcp_enviado < tcp_total) {
                int n = send(cliente, tcp_saida + tcp_enviado, tcp_total - tcp_enviado, 0);
                if (n > 0) tcp_enviado += n;
                else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                    ESP_LOGW(TAG, "Conexao TCP encerrada: %d", errno);
                    close(cliente); cliente = -1; tcp_total = tcp_enviado = usados = 0;
                }
            }
        }
        // Mede a duracao do ciclo e verifica seu prazo de 500 ms.
        int64_t fim = esp_timer_get_time();
        registrar_evento(4,(event_t){.timestamp=release,.id=new_event_id(),.source=SRC_PERIODIC}, inicio, fim);
        if (fim - inicio > 500000) ESP_LOGW(TAG, "MONITOR excedeu 500 ms");
        vTaskDelayUntil(&proximo, pdMS_TO_TICKS(500));
        release += 500000;
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
    for (;;) {
        int64_t inicio = esp_timer_get_time();
        portENTER_CRITICAL(&tempo_lock);
        if (hora_sincronizada) ciclos_sntp++;
        portEXIT_CRITICAL(&tempo_lock);
        int64_t fim = esp_timer_get_time();
        registrar_evento(5,(event_t){.timestamp=release,.id=new_event_id(),.source=SRC_PERIODIC}, inicio, fim);
        vTaskDelayUntil(&proximo, pdMS_TO_TICKS(1000));
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
#define EXAMPLE_TOUCH_CHAN_INIT_SCAN_TIMES 4

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

// Threshold = 2%
static float s_thresh2bm_ratio[EXAMPLE_TOUCH_CHANNEL_NUM] = {

    [0 ... EXAMPLE_TOUCH_CHANNEL_NUM - 1] = 0.02f};

// ======================================================
// CALLBACK - TOUCH ATIVADO
// ======================================================

static bool example_touch_on_active_cb(
    touch_sensor_handle_t sens_handle,
    const touch_active_event_data_t *event,
    void *user_ctx)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    switch (event->chan_id)
    {

    case 4:
    {
        event_t ev = {.type=EV_NAV,.timestamp=esp_timer_get_time(),.id=new_event_id(),.source=SRC_TOUCH};
        touches_nav++;


        if(xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken)!=pdTRUE) event_drop(2);

        break;
    }

    case 9:
    {
        event_t ev = {.type=EV_TEL,.timestamp=esp_timer_get_time(),.id=new_event_id(),.source=SRC_TOUCH};
        touches_tel++;

        if(xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken)!=pdTRUE) event_drop(2);

        break;
    }

    case 0:
    {
        touches_fs++;
        event_t ev = {.timestamp=esp_timer_get_time(),.id=new_event_id(),.source=SRC_TOUCH};
        if(xQueueSendFromISR(qFS,&ev,&higher_priority_task_woken)!=pdTRUE) event_drop(3);

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
    ESP_EARLY_LOGW(
        "TOUCH",
        "CH%d liberado",
        (int)event->chan_id);

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

    touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];

    // ==================================================
    // 1. Cria controller
    // ==================================================

    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();

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

    ESP_ERROR_CHECK(
        touch_sensor_start_continuous_scanning(
            sens_handle));

    ESP_LOGI(
        TAG,
        "Touch sensor pronto");

    return ESP_OK;
}

// Resumo cumulativo: nenhuma impressao ocorre dentro de uma secao critica.
static void print_metrics(void)
{
    for(unsigned i=0;i<6;i++) {
        uint32_t runs, misses, drops, windows, violations; uint64_t r,c,l,j; unsigned h;
        portENTER_CRITICAL(&metric_lock);
        timing_metrics_t *m=&metrics[i];
        runs=m->runs; misses=m->misses; drops=m->drops; windows=m->windows; violations=m->violations;
        r=m->response_max; c=m->execution_max; l=m->latency_max; j=m->jitter_max;
        h=timing_hwm99_bin(m);
        portEXIT_CRITICAL(&metric_lock);
        printf("METRIC %s runs=%lu misses=%lu drops=%lu WCRT_us=%llu C_wall_max_us=%llu latency_max_us=%llu jitter_max_us=%llu HWM99_bin_us=%u overflow=%u mk=%d/%d windows=%lu violations=%lu\n",
               metric_names[i],(unsigned long)runs,(unsigned long)misses,(unsigned long)drops,
               (unsigned long long)r,(unsigned long long)c,(unsigned long long)l,(unsigned long long)j,
               h*HIST_US,h==HIST_BINS-1,MK_M,MK_K,(unsigned long)windows,(unsigned long)violations);
    }
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    static TaskStatus_t tasks[32];
    static uint32_t last_total, last_idle[portNUM_PROCESSORS];
    uint32_t total=0, idle[portNUM_PROCESSORS]={0};
    UBaseType_t count=uxTaskGetSystemState(tasks,32,&total);
    if(count) {
        for(unsigned i=0;i<count;i++) {
            for(unsigned core=0;core<portNUM_PROCESSORS;core++)
                if(tasks[i].xHandle==xTaskGetIdleTaskHandleForCore(core)) idle[core]=tasks[i].ulRunTimeCounter;
        }
        uint32_t elapsed=total-last_total;
        for(unsigned core=0;core<portNUM_PROCESSORS;core++) {
            float busy=elapsed ? 100.0f*(1.0f-(float)(idle[core]-last_idle[core])/elapsed) : 0;
            printf("CPU core=%u busy_pct=%.2f\n",core,limit(busy,0,100));
            last_idle[core]=idle[core];
        }
        last_total=total;
    } else ESP_LOGW(TAG,"Aumente a capacidade do snapshot de tarefas para medir CPU");
#endif
    portENTER_CRITICAL(&drone_lock);
    drone_t snapshot=drone;
    portEXIT_CRITICAL(&drone_lock);
    printf("STATE roll=%.2f pitch=%.2f yaw=%.2f altitude=%.2f x=%.2f y=%.2f waypoint=%u emergency=%u motors=%.1f,%.1f,%.1f,%.1f\n",
           snapshot.estimate.roll,snapshot.estimate.pitch,snapshot.estimate.yaw,snapshot.estimate.altitude,
           snapshot.estimate.x,snapshot.estimate.y,snapshot.waypoint,snapshot.emergency,
           snapshot.motors.m1,snapshot.motors.m2,snapshot.motors.m3,snapshot.motors.m4);
}

// ======================================================
// APP MAIN
// ======================================================

void app_main(void)
{
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

    //inicializar wifi
    wifi_init();
    // IPC
    drone_init(&drone);
    qLog = xQueueCreate(1024, sizeof(log_evento_t));
    assert(qLog != NULL);
    qNav = xQueueCreate(16, sizeof(event_t));
    qFS = xQueueCreate(16, sizeof(event_t));
    qCtrl = xQueueCreate(16, sizeof(event_t));
    assert(qNav && qFS && qCtrl);

    // Cria tasks

    ESP_ERROR_CHECK(xTaskCreate(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_monitor, "MON", STK, NULL, PRIO_MON, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(task_time, "TIME", STK, NULL, PRIO_TIME, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(example_touch_init());

    ESP_LOGI(TAG, "Sistema iniciado");

    while (1)
    {

        vTaskDelay(
            pdMS_TO_TICKS(5000));
        print_metrics();
    }
}
