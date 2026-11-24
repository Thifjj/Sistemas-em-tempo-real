/*
 * Drone ESP32 + FreeRTOS
 * ESP-IDF 6.1
 */

#include "esp_random.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/touch_sens.h"
#include "touch_sens_example_config.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include <time.h>
#include <stdlib.h>
#include "lwip/sockets.h"

#define TAG "DRONE"

// ======================================================
// PERIODICIDADE E PRIORIDADES
// ======================================================

#define FUS_T_MS 5

// Protocolo de rede usado para receber comandos: 1 = TCP (porta 3334), 0 = UDP (porta 3333)
#define USE_TCP 0

#define PRIO_FS_TASK 5
#define PRIO_FUS_IMU 4
#define PRIO_CTRL_ATT 3
#define PRIO_NAV_PLAN 2

#define STK 3072

#define D_FUS_IMU 5

#define D_CTRL_ATT 5

#define D_NAV_PLAN 20

#define D_FS_TASK 500

#define D_MONITOR_TASK 500

#define MONITOR_T_MS 500

#define D_TIME_TASK 1000

#define TIME_T_MS 1000

#define WIFI_SSID "NOME_DA_REDE_24GHZ"
#define WIFI_PASSWORD "SENHA_DA_REDE"


// Timestamps em microssegundos desde o boot; contadores acumulados.
// Para NAV/FS/CTRL, guarda somente o evento mais recente.
static volatile int64_t ctrl_event_us, nav_event_us, fs_event_us;
static volatile uint32_t runs_fus, runs_ctrl, runs_nav, runs_fs, runs_mon, runs_time;
static volatile uint32_t misses_fus, misses_ctrl, misses_nav, misses_fs, misses_mon, misses_time;
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
    EV_TEL = 2,
    EV_FS = 3
} evt_t;

// Queue para NAV / Telemetria
static QueueHandle_t qNav = NULL;

// Queue de comandos recebidos pela rede -> MONITOR_TASK
static QueueHandle_t qMonitor = NULL;

// Queue de log de eventos: FS_TASK / NAV_PLAN -> MONITOR_TASK -> PC
// Cada item é uma linha de texto pronta
#define LOG_LEN 128
static QueueHandle_t qLog = NULL;

// Ciclos (segundos) da TIME_TASK desde a última sincronização SNTP
static volatile uint32_t ciclos_desde_sync = 0;

// Destino do log (preenchido pelo servidor de rede quando o PC fala com a ESP32)
#if USE_TCP
static volatile int tcp_client = -1; // socket do cliente conectado (-1 = ninguém)
#else
static volatile int udp_sock = -1;
static struct sockaddr_in log_dest; // endereço do PC (última mensagem recebida)
static volatile int log_dest_ok = 0;
#endif

// Semáforo para Fail-Safe
static SemaphoreHandle_t semFS = NULL;

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
    g_esc.m1 = m1;
    g_esc.m2 = m2;
    g_esc.m3 = m3;
    g_esc.m4 = m4;
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

        // Ângulos aleatórios em [-10, 10] graus: com os ganhos do controle
        // (kp 0.8/0.8/0.4) e throttle 50, os motores ficam sempre em [30, 70]%
        g_state.roll = ((float)(esp_random() % 2001) - 1000.0f) / 100.0f;
        g_state.pitch = ((float)(esp_random() % 2001) - 1000.0f) / 100.0f;
        g_state.yaw = ((float)(esp_random() % 2001) - 1000.0f) / 100.0f;

        // Simula aproximadamente 1 ms
        cpu_tight_loop_us(1000);

        // Acorda CTRL_ATT
        if (hCTRL)
        {
            ctrl_event_us = esp_timer_get_time();
            xTaskNotifyGive(hCTRL);
        }
        int64_t t_fim = esp_timer_get_time();
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
    evt_t ev;
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
            runs_nav++;
            misses_nav += task_end_us - event_us > D_NAV_PLAN * 1000LL;
            printf("NAV_PLAN: evento=%.2fms inicio=%.2fms fim=%.2fms lat_touch=%.2fms misses=%lu/%lu\n",
                   event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                   (task_start_us - event_us) / 1000.0, (unsigned long)misses_nav, (unsigned long)runs_nav);

            // Log para o PC (timeout 0: se a queue estiver cheia, descarta e não atrasa a task)
            char log[LOG_LEN];
            snprintf(log, sizeof(log), "NAV_PLAN ev=%d evento=%.2fms inicio=%.2fms fim=%.2fms misses=%lu/%lu\n",
                     (int)ev, event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                     (unsigned long)misses_nav, (unsigned long)runs_nav);
            xQueueSend(qLog, log, 0);
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
            runs_fs++;
            misses_fs += task_end_us - event_us > D_FS_TASK * 1000LL;

            printf("FAIL-SAFE! tratado em %.2f ms\n", dt / 1000.0);
            printf("FS_TASK: evento=%.2fms inicio=%.2fms fim=%.2fms lat_touch=%.2fms misses=%lu/%lu\n",
                   event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                   (task_start_us - event_us) / 1000.0, (unsigned long)misses_fs, (unsigned long)runs_fs);

            // Log para o PC (timeout 0: se a queue estiver cheia, descarta e não atrasa a task)
            char log[LOG_LEN];
            snprintf(log, sizeof(log), "FS_TASK evento=%.2fms inicio=%.2fms fim=%.2fms misses=%lu/%lu\n",
                     event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                     (unsigned long)misses_fs, (unsigned long)runs_fs);
            xQueueSend(qLog, log, 0);
        }
    }
}

// ======================================================
// TASK WIFI
// ======================================================

static void conectar_wifi(void)
{
    // NVS: o driver WiFi guarda dados de calibração nela
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    if (ip_info.ip.addr != 0)
    {
        printf("WIFI: conectado! IP da ESP32: " IPSTR "\n", IP2STR(&ip_info.ip));
    }
    else
    {
        printf("WIFI: nao conseguiu IP (confira SSID/senha e se a rede e 2.4GHz)\n");
    }
}

// ======================================================
// TASK UDP_SERVER
// ======================================================

static void udp_server(void *arg)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int sock = udp_sock;

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // aceita de qualquer interface
    addr.sin_port = htons(3333);              // porta que o notebook vai usar
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    char rx[64];
    struct sockaddr_in origem;
    socklen_t origem_len = sizeof(origem);

    for (;;)
    {
        int len = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&origem, &origem_len);
        if (len > 0)
        {
            rx[len] = 0;
            printf("UDP: recebeu '%s'\n", rx);

            // Guarda o endereço do PC: é para lá que a monitorem tese agora ta implemetado entao envia o log
            log_dest = origem;
            log_dest_ok = 1;

            evt_t cmd = 0;
            if (strncmp(rx, "NAV", 3) == 0)
            {
                cmd = EV_NAV;
            }
            else if (strncmp(rx, "TEL", 3) == 0)
            {
                cmd = EV_TEL;
            }
            else if (strncmp(rx, "FS", 2) == 0)
            {
                cmd = EV_FS;
            }

            if (cmd != 0)
            {
                xQueueSend(qMonitor, &cmd, 0); // timeout 0: a task de rede nunca fica presa
                sendto(sock, "OK\n", 3, 0, (struct sockaddr *)&origem, origem_len);
            }
            else
            {
                sendto(sock, "INVALIDO\n", 9, 0, (struct sockaddr *)&origem, origem_len);
            }
        }
    }
}

// ======================================================
// TASK TCP_SERVER
// ======================================================

static void tcp_server(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(3334); // porta TCP (a UDP usa 3333)
    bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr));
    listen(listen_sock, 1);

    char rx[64];

    for (;;)
    {
        // Espera o notebook conectar
        int cliente = accept(listen_sock, NULL, NULL);
        if (cliente < 0)
        {
            continue;
        }
        printf("TCP: cliente conectado\n");
        tcp_client = cliente; // a monitor envia o log por este socket

        // Lê comandos até o notebook fechar a conexão (recv <= 0)
        int len = recv(cliente, rx, sizeof(rx) - 1, 0);
        while (len > 0)
        {
            rx[len] = 0;
            printf("TCP: recebeu '%s'\n", rx);

            evt_t cmd = 0;
            if (strncmp(rx, "NAV", 3) == 0)
            {
                cmd = EV_NAV;
            }
            else if (strncmp(rx, "TEL", 3) == 0)
            {
                cmd = EV_TEL;
            }
            else if (strncmp(rx, "FS", 2) == 0)
            {
                cmd = EV_FS;
            }

            if (cmd != 0)
            {
                xQueueSend(qMonitor, &cmd, 0);
                send(cliente, "OK\n", 3, 0);
            }
            else
            {
                send(cliente, "INVALIDO\n", 9, 0);
            }

            len = recv(cliente, rx, sizeof(rx) - 1, 0);
        }

        tcp_client = -1; // avisa a monitor antes de fechar
        close(cliente);
        printf("TCP: cliente desconectou\n");
    }
}
// ======================================================
// TASK MONITOR TASK
// ======================================================

// Deadline 500ms, Periodicidade 100ms e c: ?
static void MONITOR_TASK(void *arg)
{
    TickType_t next = xTaskGetTickCount();

    const TickType_t T = pdMS_TO_TICKS(MONITOR_T_MS);
    int64_t release_us = esp_timer_get_time();

    evt_t cmd;
    char linha[LOG_LEN];
    for (;;)
    {
        int64_t t_inicio = esp_timer_get_time();
        int processados = 0;

        // Processa todos os comandos que chegaram desde o último período (sem bloquear)
        while (xQueueReceive(qMonitor, &cmd, 0) == pdTRUE)
        {
            if (cmd == EV_FS)
            {
                fs_event_us = esp_timer_get_time();
                xSemaphoreGive(semFS);
            }
            else if (cmd == EV_NAV || cmd == EV_TEL)
            {
                nav_event_us = esp_timer_get_time();
                xQueueSend(qNav, &cmd, 0);
            }
            else
            {
                printf("MONITOR: comando invalido\n");
            }
            processados++;
        }

        // Envia ao PC os logs acumulados (no máximo 8 por ciclo para limitar o tempo)
        int enviados = 0;
        while (enviados < 8 && xQueueReceive(qLog, linha, 0) == pdTRUE)
        {
#if USE_TCP
            int cliente = tcp_client;
            if (cliente >= 0)
            {
                send(cliente, linha, strlen(linha), MSG_DONTWAIT);
            }
#else
            if (log_dest_ok && udp_sock >= 0)
            {
                sendto(udp_sock, linha, strlen(linha), 0, (struct sockaddr *)&log_dest, sizeof(log_dest));
            }
#endif
            enviados++;
        }

        int64_t t_fim = esp_timer_get_time();
        runs_mon++;
        misses_mon += t_fim - release_us > D_MONITOR_TASK * 1000LL;

        printf("MONITOR: cmds=%d evento=%.2fms inicio=%.2fms fim=%.2fms exec=%.3fms misses=%lu/%lu\n",
               processados, release_us / 1000.0, t_inicio / 1000.0, t_fim / 1000.0,
               (t_fim - t_inicio) / 1000.0, (unsigned long)misses_mon, (unsigned long)runs_mon);

        // Mantém período de 500 ms
        vTaskDelayUntil(&next, T);
        release_us += T * portTICK_PERIOD_MS * 1000LL;
    }
}

// ======================================================
// TASK 6 - TIME TASK
// Periódica: 1 s (D = 1 s) - soft
// Captura a hora via SNTP e conta ciclos desde a última atualização
// ======================================================

// Chamado pelo SNTP (task do lwIP) a cada sincronização: só zera o contador
static void sntp_sync_cb(struct timeval *tv)
{
    ciclos_desde_sync = 0;
}

static void TIME_TASK(void *arg)
{
    // Fuso do Brasil (UTC-3); sem isso a hora sai em UTC
    setenv("TZ", "<-03>3", 1);
    tzset();

    // Cliente SNTP (a rede já tem IP: conectar_wifi roda antes)
    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    sntp_cfg.sync_cb = sntp_sync_cb;
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_cfg));

    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(TIME_T_MS);
    int64_t release_us = esp_timer_get_time();

    for (;;)
    {
        int64_t t_inicio = esp_timer_get_time();

        ciclos_desde_sync++;

        time_t agora = time(NULL);
        struct tm hora;
        localtime_r(&agora, &hora);

        int64_t t_fim = esp_timer_get_time();
        runs_time++;
        misses_time += t_fim - release_us > D_TIME_TASK * 1000LL;

        // Ano 1970 = ainda nunca sincronizou
        if (hora.tm_year + 1900 < 2024)
        {
            printf("TIME_TASK: sem sincronizacao SNTP | ciclos_desde_sync=%lu", (unsigned long)ciclos_desde_sync);
        }
        else
        {
            printf("TIME_TASK: %02d/%02d/%04d %02d:%02d:%02d | ciclos_desde_sync=%lu",
                   hora.tm_mday, hora.tm_mon + 1, hora.tm_year + 1900,
                   hora.tm_hour, hora.tm_min, hora.tm_sec, (unsigned long)ciclos_desde_sync);
        }
        printf(" | evento=%.2fms inicio=%.2fms fim=%.2fms exec=%.3fms misses=%lu/%lu\n",
               release_us / 1000.0, t_inicio / 1000.0, t_fim / 1000.0,
               (t_fim - t_inicio) / 1000.0, (unsigned long)misses_time, (unsigned long)runs_time);

        // Log para o PC (timeout 0: se a queue estiver cheia, descarta e não atrasa a task)
        char log[LOG_LEN];
        if (hora.tm_year + 1900 < 2024)
        {
            snprintf(log, sizeof(log), "TIME_TASK sem sincronizacao SNTP ciclos_desde_sync=%lu misses=%lu/%lu\n",
                     (unsigned long)ciclos_desde_sync, (unsigned long)misses_time, (unsigned long)runs_time);
        }
        else
        {
            snprintf(log, sizeof(log), "TIME_TASK %02d/%02d/%04d %02d:%02d:%02d ciclos_desde_sync=%lu misses=%lu/%lu\n",
                     hora.tm_mday, hora.tm_mon + 1, hora.tm_year + 1900,
                     hora.tm_hour, hora.tm_min, hora.tm_sec, (unsigned long)ciclos_desde_sync,
                     (unsigned long)misses_time, (unsigned long)runs_time);
        }
        xQueueSend(qLog, log, 0);

        // Mantém período de 1 s
        vTaskDelayUntil(&next, T);
        release_us += T * portTICK_PERIOD_MS * 1000LL;
    }
}

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
        evt_t ev = EV_NAV;
        touches_nav++;
        nav_event_us = esp_timer_get_time();

        xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken);

        break;
    }

    case 9:
    {
        evt_t ev = EV_TEL;
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
    conectar_wifi();

    // IPC
    qNav = xQueueCreate(8, sizeof(evt_t));
    semFS = xSemaphoreCreateBinary();
    qMonitor = xQueueCreate(8, sizeof(evt_t));
    qLog = xQueueCreate(16, LOG_LEN);

    // Cria tasks

    xTaskCreatePinnedToCore(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS, 0);
    xTaskCreatePinnedToCore(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL, 0);
    xTaskCreatePinnedToCore(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV, 0);
    xTaskCreatePinnedToCore(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS, 0);
    xTaskCreatePinnedToCore(MONITOR_TASK, "MONITOR", STK, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TIME_TASK, "TIME", 4096, NULL, 1, NULL, 1);

    // Cria só o servidor escolhido em USE_TCP (a outra task não ocupa RAM)
#if USE_TCP
    xTaskCreatePinnedToCore(tcp_server, "TCP_SRV", 4096, NULL, 1, NULL, 1);
#else
    xTaskCreatePinnedToCore(udp_server, "UDP_SRV", 4096, NULL, 1, NULL, 1);
#endif

    ESP_ERROR_CHECK(example_touch_init());

    ESP_LOGI(TAG, "Sistema iniciado");

    while (1)
    {

        vTaskDelay(
            pdMS_TO_TICKS(1000));
        printf("TOTAL: hard_misses=%lu (FUS=%lu CTRL=%lu FS=%lu) soft_misses=%lu (NAV=%lu MON=%lu TIME=%lu)\n",
               (unsigned long)(misses_fus + misses_ctrl + misses_fs),
               (unsigned long)misses_fus, (unsigned long)misses_ctrl, (unsigned long)misses_fs,
               (unsigned long)(misses_nav + misses_mon + misses_time), (unsigned long)misses_nav, (unsigned long)misses_mon,
               (unsigned long)misses_time);
        printf("TOUCH: CH4=%lu CH9=%lu CH0=%lu | tasks NAV=%lu FS=%lu\n",
               (unsigned long)touches_nav, (unsigned long)touches_tel, (unsigned long)touches_fs,
               (unsigned long)runs_nav, (unsigned long)runs_fs);
    }
}