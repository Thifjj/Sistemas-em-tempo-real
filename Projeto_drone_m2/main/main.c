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
#include "freertos/semphr.h"
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
#define STK 3072

#define D_FUS_IMU 5

#define D_CTRL_ATT 5

#define D_NAV_PLAN 20

#define D_FS_TASK 10

// Timestamps em microssegundos desde o boot; contadores acumulados.
// Para NAV/FS/CTRL, guarda somente o evento mais recente.
static volatile int64_t ctrl_event_us, nav_event_us, fs_event_us;
static volatile uint32_t runs_fus, runs_ctrl, runs_nav, runs_fs;
static volatile uint32_t misses_fus, misses_ctrl, misses_nav, misses_fs;
static volatile uint32_t touches_nav, touches_tel, touches_fs;

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

// Queue para NAV / Telemetria
static QueueHandle_t qNav = NULL;

// Semáforo para Fail-Safe
static SemaphoreHandle_t semFS = NULL;
// Fila limitada: tarefas nunca esperam pela rede; perdas ficam contabilizadas.
typedef struct { const char *task; int64_t evento, inicio, fim; } log_evento_t;
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
static void registrar_evento(const char *task, int64_t evento, int64_t inicio, int64_t fim)
{
    log_evento_t item = {task, evento, inicio, fim};
    if (xQueueSend(qLog, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&log_lock); logs_perdidos++; portEXIT_CRITICAL(&log_lock);
    }
}

// ======================================================
// ESTADO SIMULADO DO DRONE
// ======================================================

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} state_t;

static state_t g_state = {
    0,
    0,
    0};

typedef struct
{
    float m1, m2, m3, m4;
} esc_output_t;

static esc_output_t g_esc = {0};

static void esc_write_simulated(float m1, float m2, float m3, float m4)
{
    g_esc.m1 = m1 < 0.0f ? 0.0f : (m1 > 100.0f ? 100.0f : m1);
    g_esc.m2 = m2 < 0.0f ? 0.0f : (m2 > 100.0f ? 100.0f : m2);
    g_esc.m3 = m3 < 0.0f ? 0.0f : (m3 > 100.0f ? 100.0f : m3);
    g_esc.m4 = m4 < 0.0f ? 0.0f : (m4 > 100.0f ? 100.0f : m4);
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

    const TickType_t T = pdMS_TO_TICKS(FUS_T_MS);
    int64_t release_us = esp_timer_get_time();

    for (;;)
    {
        // pegar tempo inicial
        int64_t t_inicio = esp_timer_get_time();
        // Simulação da fusão dos sensores

        g_state.roll *= 0.98f;
        g_state.pitch *= 0.98f;
        g_state.yaw *= 0.98f;
        g_state.roll += 0.1f;
        g_state.yaw += 0.05f;

        // Simula aproximadamente 1 ms
        cpu_tight_loop_us(1000);

        // Acorda CTRL_ATT
        if (hCTRL)
        {
            ctrl_event_us = esp_timer_get_time();
            xTaskNotifyGive(hCTRL);
        }
        int64_t t_fim = esp_timer_get_time();
        registrar_evento("FUS_IMU", release_us, t_inicio, t_fim);
        runs_fus++;
        misses_fus += t_fim - release_us > D_FUS_IMU * 1000LL;
        int64_t tempo = (t_fim - t_inicio) / 1000;

        // UART a 115200 baud nao acompanha um log longo a cada 5 ms.
        if (runs_fus % 100 == 0)
        {
            printf("FUS_IMU: %lld ms", (long long)tempo);
            printf(" | evento=%.2fms inicio=%.2fms fim=%.2fms exec=%.2fms misses=%lu/%lu\n",
                   release_us / 1000.0, t_inicio / 1000.0, t_fim / 1000.0,
                   (t_fim - t_inicio) / 1000.0, (unsigned long)misses_fus, (unsigned long)runs_fus);
        }

        // Mantém período de 5 ms
        vTaskDelayUntil(&next, T);
        release_us += T * portTICK_PERIOD_MS * 1000LL;
    }
}

// ======================================================
// TASK 2 - CONTROLE DE ATITUDE
// ======================================================

// ====== Controle de atitude (espera notificação da FUS_IMU) ======

// HARD - tem acao principal de controle dos atuadores e do PID se atualizar usando valores muito antigos drone pode cair bater em alguma coisa e ja era
//  T-indefinido pela task fus_imu - D=5ms C: medir tempo de computacao da task
static void task_ctrl_att(void *arg)
{
    const float roll_ref = 0.0f;
    const float pitch_ref = 0.0f;
    const float yaw_ref = 0.0f;

    const float kp_roll = 0.8f;
    const float kp_pitch = 0.8f;
    const float kp_yaw = 0.4f;

    const float throttle_base = 50.0f;

    static int count = 0;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // acorda quando FUS_IMU terminar
        int64_t event_us = ctrl_event_us;
        int64_t task_start_us = esp_timer_get_time();

        // calcula erro de altitude
        float error_roll = roll_ref - g_state.roll;
        float error_pitch = pitch_ref - g_state.pitch;
        float error_yaw = yaw_ref - g_state.yaw;

        // controlador simplificado

        float ctrl_roll = kp_roll * error_roll;
        float ctrl_pitch = kp_pitch * error_pitch;
        float ctrl_yaw = kp_yaw * error_yaw;

        float m1 = throttle_base + ctrl_pitch + ctrl_roll - ctrl_yaw;
        float m2 = throttle_base + ctrl_pitch - ctrl_roll + ctrl_yaw;
        float m3 = throttle_base - ctrl_pitch - ctrl_roll - ctrl_yaw;
        float m4 = throttle_base - ctrl_pitch + ctrl_roll + ctrl_yaw;

        esc_write_simulated(m1, m2, m3, m4);

        // imprime a cada 100 execuções
        count++;

        if (count >= 100)
        {
            printf(
                "ESC: M1=%.1f%% M2=%.1f%% M3=%.1f%% M4=%.1f%%\n", g_esc.m1, g_esc.m2, g_esc.m3, g_esc.m4);

            count = 0;
        }

        // PID simulado + "carga" ~0.8 ms
        // (nesta demo, apenas consome tempo previsível)
        cpu_tight_loop_us(800);
        int64_t task_end_us = esp_timer_get_time();
        registrar_evento("CTRL_ATT", event_us, task_start_us, task_end_us);
        runs_ctrl++;
        misses_ctrl += task_end_us - event_us > D_CTRL_ATT * 1000LL;
        if (runs_ctrl % 100 == 0)
        {
            printf("CTRL_ATT: evento=%.2fms inicio=%.2fms fim=%.2fms exec=%.2fms misses=%lu/%lu\n",
                   event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                   (task_end_us - task_start_us) / 1000.0, (unsigned long)misses_ctrl, (unsigned long)runs_ctrl);
        }
    }
}

// ======================================================
// TASK 3 - NAVEGAÇÃO / TELEMETRIA
// ======================================================
// SOFT - PQ nao compromete o funcionamento do sistema e nem nada fisico, so teria um delay entre trocar de rotas nao teria problema critico ao estourar deadline
// T - touch4 , D = 20ms c: deve ser medido
static void task_nav_plan(void *arg)
{
    nav_evt_t ev;
    for (;;)
    {
        // Fica bloqueada esperando algo entrar na Queue
        if (
            xQueueReceive(qNav, &ev, portMAX_DELAY) == pdTRUE)
        {
            int64_t event_us = nav_event_us;
            int64_t task_start_us = esp_timer_get_time();
            // --------------------------
            // Evento de navegação
            // --------------------------
            if (ev == EV_NAV)
            {
                // Simula cálculo de rota
                // aproximadamente 3.5 ms
                ESP_LOGI(
                    "NAV_PLAN",
                    "Evento NAV recebido");
                cpu_tight_loop_us(3500);
            }

            // --------------------------
            // Evento de telemetria
            // --------------------------

            else if (ev == EV_TEL)
            {

                printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f\n", g_state.roll, g_state.pitch, g_state.yaw);

                // Simula processamento
                cpu_tight_loop_us(500);
            }
            int64_t task_end_us = esp_timer_get_time();
            registrar_evento("NAV_PLAN", event_us, task_start_us, task_end_us);
            runs_nav++;
            misses_nav += task_end_us - event_us > D_NAV_PLAN * 1000LL;
            printf("NAV_PLAN: evento=%.2fms inicio=%.2fms fim=%.2fms lat_touch=%.2fms misses=%lu/%lu\n",
                   event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                   (task_start_us - event_us) / 1000.0, (unsigned long)misses_nav, (unsigned long)runs_nav);
        }
    }
}

// ======================================================
// TASK 4 - FAIL SAFE
// ======================================================
//
// HARD - acao que nao pode falhar se nao perde tudo
// T - touch 0, D = 10ms c: medir
static void task_fail_safe(void *arg)
{
    // O LED inicia apagado e permanece aceso apos uma emergencia.
    gpio_set_level(BLINK_GPIO, 0);
    for (;;)
    {

        // Fica bloqueada esperando o semáforo
        if (xSemaphoreTake(semFS, portMAX_DELAY) == pdTRUE)
        {
            int64_t event_us = fs_event_us;
            int64_t task_start_us = esp_timer_get_time();
            int64_t t0 = esp_timer_get_time();
            // Simula ação crítica:
            // reduzir throttle / hover / pouso

            cpu_tight_loop_us(900);

            int64_t dt = esp_timer_get_time() - t0;
            int64_t task_end_us = esp_timer_get_time();
            registrar_evento("FS_TASK", event_us, task_start_us, task_end_us);
            runs_fs++;
            misses_fs += task_end_us - event_us > D_FS_TASK * 1000LL;

            printf("FAIL-SAFE! tratado em %.2f ms\n", dt / 1000.0);
            printf("FS_TASK: evento=%.2fms inicio=%.2fms fim=%.2fms lat_touch=%.2fms misses=%lu/%lu\n",
                   event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                   (task_start_us - event_us) / 1000.0, (unsigned long)misses_fs, (unsigned long)runs_fs);
        }
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
// A resposta indica se o evento foi aceito pela fila ou semaforo.
static const char *monitor_command(const char *cmd)
{
    const char *espaco = strchr(cmd, ' ');
    if (espaco) cmd = espaco + 1;
    if (strcmp(cmd, "NAV") == 0) {
        nav_evt_t ev = EV_NAV;
        nav_event_us = esp_timer_get_time();
        return xQueueSend(qNav, &ev, 0) == pdTRUE ? "OK\n" : "FILA_CHEIA\n";
    }
    if (strcmp(cmd, "FS") == 0) {
        esp_timer_stop(led_timer);
        gpio_set_level(BLINK_GPIO, 1);
        ESP_ERROR_CHECK(esp_timer_start_once(led_timer, 1000000));
        fs_event_us = esp_timer_get_time();
        return xSemaphoreGive(semFS) == pdTRUE ? "OK\n" : "PENDENTE\n";
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
    int udp = -1, servidor = -1, cliente = -1;
    char entrada[256];
    size_t usados = 0;
    uint32_t sequencia = 0;
    char tcp_saida[384];
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
            for (int i = 0; i < 64; i++) {
                if (cliente >= 0 && tcp_enviado < tcp_total) break;
                log_evento_t item;
                if (xQueueReceive(qLog, &item, 0) != pdTRUE) break;
                struct timeval agora; gettimeofday(&agora, NULL);
                portENTER_CRITICAL(&tempo_lock);
                bool sincronizado = hora_sincronizada; uint32_t ciclos = ciclos_sntp;
                portEXIT_CRITICAL(&tempo_lock);
                portENTER_CRITICAL(&log_lock); uint32_t perdas = logs_perdidos; portEXIT_CRITICAL(&log_lock);
                char log[384];
                int n = snprintf(log, sizeof(log),
                    "{\"seq\":%" PRIu32 ",\"epoch_ms\":%lld,\"sntp\":%s,\"ciclos_sntp\":%" PRIu32 ",\"task\":\"%s\",\"evento_us\":%lld,\"inicio_us\":%lld,\"fim_us\":%lld,\"perdidos\":%" PRIu32 "}\n",
                    sequencia++, (long long)agora.tv_sec * 1000 + agora.tv_usec / 1000,
                    sincronizado ? "true" : "false", ciclos, item.task,
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
        registrar_evento("MONITOR_TASK", inicio, inicio, fim);
        if (fim - inicio > 500000) ESP_LOGW(TAG, "MONITOR excedeu 500 ms");
        vTaskDelayUntil(&proximo, pdMS_TO_TICKS(500));
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
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_setservername(0, "a.st1.ntp.br");
    sntp_setservername(1, "b.st1.ntp.br");
    sntp_setservername(2, "pool.ntp.org");
    sntp_init();

    // Periodicidade de um segundo; o callback reinicia o contador de ciclos.
    TickType_t proximo = xTaskGetTickCount();
    for (;;) {
        int64_t inicio = esp_timer_get_time();
        portENTER_CRITICAL(&tempo_lock);
        if (hora_sincronizada) ciclos_sntp++;
        portEXIT_CRITICAL(&tempo_lock);
        int64_t fim = esp_timer_get_time();
        registrar_evento("TIME_TASK", inicio, inicio, fim);
        vTaskDelayUntil(&proximo, pdMS_TO_TICKS(1000));
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
        nav_evt_t ev = EV_NAV;
        touches_nav++;
        nav_event_us = esp_timer_get_time();

        xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken);

        break;
    }

    case 9:
    {
        nav_evt_t ev = EV_TEL;
        touches_tel++;
        nav_event_us = esp_timer_get_time();
        xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken);

        break;
    }

    case 0:
    {
        touches_fs++;
        fs_event_us = esp_timer_get_time();
        xSemaphoreGiveFromISR(semFS, &higher_priority_task_woken);

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
    qLog = xQueueCreate(512, sizeof(log_evento_t));
    assert(qLog != NULL);
    qNav = xQueueCreate(8, sizeof(nav_evt_t));
    semFS = xSemaphoreCreateBinary();

    // Cria tasks

    xTaskCreate(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS);
    xTaskCreate(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL);
    xTaskCreate(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV);
    xTaskCreate(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS);
    xTaskCreate(task_monitor, "MON", STK, NULL, PRIO_MON, NULL);
    xTaskCreate(task_time, "TIME", STK, NULL, PRIO_TIME, NULL);

    ESP_ERROR_CHECK(example_touch_init());

    ESP_LOGI(TAG, "Sistema iniciado");

    while (1)
    {

        vTaskDelay(
            pdMS_TO_TICKS(1000));
        printf("TOTAL: hard_misses=%lu (FUS=%lu CTRL=%lu FS=%lu) soft_misses=%lu (NAV=%lu)\n",
               (unsigned long)(misses_fus + misses_ctrl + misses_fs),
               (unsigned long)misses_fus, (unsigned long)misses_ctrl, (unsigned long)misses_fs,
               (unsigned long)misses_nav, (unsigned long)misses_nav);
        printf("TOUCH: CH4=%lu CH9=%lu CH0=%lu | tasks NAV=%lu FS=%lu\n",
               (unsigned long)touches_nav, (unsigned long)touches_tel, (unsigned long)touches_fs,
               (unsigned long)runs_nav, (unsigned long)runs_fs);
    }
}
