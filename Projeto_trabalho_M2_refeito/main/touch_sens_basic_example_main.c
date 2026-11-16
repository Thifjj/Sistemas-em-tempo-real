/*
 * Drone ESP32 + FreeRTOS
 * ESP-IDF 6.1
 *
 * ============================================================
 * CHECKLIST - TRABALHO M2 SISTEMAS EM TEMPO REAL
 * ============================================================
 *
 * [x] 1. IMPLEMENTACAO DAS TASKS
 *     [x] FUS_IMU - Periodica (T=5ms, D=5ms)
 *     [x] CTRL_ATT - Acionada pela FUS_IMU (D=5ms)
 *     [x] NAV_PLAN - Acionada por evento (D=20ms)
 *     [x] FS_TASK - Acionada por evento (D=10ms)
 *     [x] MONITOR_TASK - Comunicacao TCP/UDP (D=500ms)
 *     [x] TIME_TASK - Sincronizacao SNTP (D=1s)
 *
 * [x] 2. COMUNICACAO E SENSORES
 *     [x] Configuracao Wi-Fi da ESP32
 *     [x] Implementacao dos servidores TCP e UDP
 *     [x] Recebimento dos comandos "fs" e "nav"
 *     [x] Acionamento das tasks sem ativacao permanente
 *     [x] Envio de logs de eventos para o PC
 *     [x] Configuracao dos sensores touch
 * 
 * [x] 2A. TIME_TASK - SNTP
 *     [x] Sincronizacao do relogio com servidor SNTP
 *     [x] Contagem dos ciclos desde a ultima sincronizacao
 *
 * ============================================================
 * PENDENCIAS
 * ============================================================
 *
 * [ ] 2B. WCRT E HWM (EM ANDAMENTO)
 *     [ ] Medir tempo de resposta de todas as 6 tasks
 *     [ ] Registrar maior tempo de resposta (WCRT)
 *     [ ] Calcular HWM(100%) e HWM(99%)
 *     [ ] Registrar timestamps de evento, inicio e fim
 *     [ ] Medir tempo de computacao (C) de cada task
 *     [ ] Verificar deadlines e contabilizar misses
 *
 * [ ] 2C. MODELO (m,k)-FIRM
 *     [ ] Implementar analise da NAV_PLAN (soft real-time)
 *     [ ] Utilizar janelas de 10 execucoes consecutivas
 *     [ ] Verificar atendimento de pelo menos 9 deadlines
 *     [ ] Contabilizar janelas que violam (9,10)-firm
 *
 * [ ] 2D. UTILIZACAO DA CPU
 *     [ ] Medir ocupacao da CPU
 *     [ ] Separar tempo de execucao e tempo bloqueado
 *     [ ] Registrar resultados para cada configuracao
 *
 * [ ] 2E. LOGS E TIMESTAMPS TCP/UDP
 *     [ ] Adicionar numero de sequencia aos pacotes
 *     [ ] Identificar protocolo utilizado (TCP ou UDP)
 *     [ ] Registrar tamanho dos pacotes
 *     [ ] Registrar timestamps sincronizados via SNTP
 *     [ ] Registrar momento de envio e recebimento
 *     [ ] Salvar resultados para analise posterior
 *
 * [ ] 2F. MEDICAO DE RTT
 *     [ ] Medir RTT no Python
 *     [ ] Medir latencia da comunicacao TCP
 *     [ ] Medir latencia da comunicacao UDP
 *     [ ] Calcular media, maximo e variabilidade
 *     [ ] Exportar resultados para CSV
 *
 * [ ] 2G. ANALISE TEMPORAL
 *     [ ] Medir latencia ISR -> task (touch)
 *     [ ] Verificar interferencia entre tasks
 *     [ ] Identificar bloqueios e inversao de prioridade
 *     [ ] Justificar classificacoes hard e soft
 *     [ ] Verificar atendimento dos deadlines
 *
 * [ ] 2H. TESTES EXPERIMENTAIS
 *     [ ] Recuperar melhor configuracao da M1
 *     [ ] Recuperar pior configuracao da M1
 *     [ ] Testar melhor configuracao utilizando TCP
 *     [ ] Testar melhor configuracao utilizando UDP
 *     [ ] Testar pior configuracao utilizando TCP
 *     [ ] Testar pior configuracao utilizando UDP
 *     [ ] Aplicar configuracoes de CPU e escalonador
 *     [ ] Testar configuracao unicore conforme enunciado
 *     [ ] Comparar resultados e identificar melhor protocolo
 *
 * [ ] 2I. CORRECOES E MELHORIAS
 *     [ ] Garantir timestamps corretos para cada evento
 *     [ ] Evitar sobrescrita de timestamps entre eventos
 *     [ ] Validar acionamentos consecutivos de FS e NAV
 *     [ ] Validar funcionamento dos logs TCP/UDP
 *     [ ] Verificar deadlines da MONITOR_TASK e TIME_TASK
 *     [ ] Validar comportamento de reconexao Wi-Fi
 *
 * [ ] 3. PONTO EXTRA - LED GPIO2
 *     [ ] Configurar GPIO2 como saida
 *     [ ] Acender LED quando comando "fs" chegar pelo PC
 *     [ ] Validar acionamento por TCP e UDP
 *
 * [ ] 4. RELATORIO FINAL
 *     [ ] Apresentar metodologia e arquitetura do sistema
 *     [ ] Descrever as 6 tasks e suas prioridades
 *     [ ] Justificar requisitos hard e soft
 *     [ ] Apresentar WCRT e HWM
 *     [ ] Apresentar resultados (m,k)-firm
 *     [ ] Apresentar misses hard e soft
 *     [ ] Apresentar latencias de touch e comunicacao
 *     [ ] Apresentar utilizacao de CPU
 *     [ ] Comparar TCP e UDP
 *     [ ] Comparar melhor e pior configuracao da M1
 *     [ ] Analisar interferencias e bloqueios
 *     [ ] Montar tabelas comparativas dos experimentos
 *     [ ] Apresentar conclusoes baseadas nas medicoes
 *
 * [ ] 5. VALIDACAO FINAL
 *     [ ] Compilar sem erros
 *     [ ] Testar todas as tasks na ESP32
 *     [ ] Testar touch e comandos TCP/UDP
 *     [ ] Conferir os resultados das medicoes
 *     [ ] Atualizar codigo no GitHub
 *     [ ] Finalizar relatorio PDF
 *     [ ] Preparar demonstracao presencial
 *
 * ============================================================
 * STATUS ATUAL:
 * 2A CONCLUIDA
 * 2B EM ANDAMENTO
 *
 * Atualizar [ ] para [x] conforme cada etapa for concluida.
 * ============================================================
 */

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/touch_sens.h"
#include "touch_sens_example_config.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_netif_sntp.h"
#include "conexao_wifi.h"
// #include "tcp_udp_server.h"

#define TAG "DRONE"

// ======================================================
// PERIODICIDADE E PRIORIDADES
// ======================================================

#define FUS_T_MS 5
#define TIME_T_MS 1000
#define MONITOR_T_MS 500

#define PRIO_FS_TASK 5
#define PRIO_FUS_IMU 4
#define PRIO_CTRL_ATT 3
#define PRIO_NAV_PLAN 2
#define PRIO_TIME 1
#define PRIO_MONITOR 3

#define STK 3072

#define D_FUS_IMU 5

#define D_CTRL_ATT 5

#define D_NAV_PLAN 20

#define D_FS_TASK 10

#define D_TIME_TASK 1000

// defines para com/monitor
#define MONITOR_UDP_PORT 6010
#define MONITOR_TCP_PORT 5000
#define MONITOR_BUFFER 128

#define SNTP_SERVER "pool.ntp.org"

static const char *SNTP_TAG = "SNTP";

// Timestamps em microssegundos desde o boot; contadores acumulados.
// Para NAV/FS/CTRL, guarda somente o evento mais recente.
static volatile int64_t ctrl_event_us, nav_event_us, fs_event_us, fus_imu_event_us, time_event_us, monitor_event_us;
static volatile uint32_t runs_fus, runs_ctrl, runs_nav, runs_fs, runs_time, runs_monitor;
static volatile uint32_t misses_fus, misses_ctrl, misses_nav, misses_fs, misses_time, misses_monitor;
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

// struct para buffer do monitor
typedef struct
{
    char texto[MONITOR_BUFFER];
} monitor_log_t;

static QueueHandle_t qMonitorLog = NULL;

static void monitor_log(const char *mensagem)
{
    if (qMonitorLog == NULL || mensagem == NULL)
        return;

    monitor_log_t evento = {0};
    snprintf(evento.texto, sizeof(evento.texto), "LOG: %s\n", mensagem);
    xQueueSend(qMonitorLog, &evento, 0);
}

static int64_t monitor_epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// fim parte auxiliares monitor

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
            runs_nav++;
            misses_nav += task_end_us - event_us > D_NAV_PLAN * 1000LL;
            printf("NAV_PLAN: evento=%.2fms inicio=%.2fms fim=%.2fms lat_touch=%.2fms misses=%lu/%lu\n",
                   event_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
                   (task_start_us - event_us) / 1000.0, (unsigned long)misses_nav, (unsigned long)runs_nav);
            monitor_log("NAV_PLAN executada");
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
            monitor_log("FS_TASK executada");
        }
    }
}

// TASK_TIME 5 - Sincronizacao SNTP
static void task_time(void *arg)
{
    uint32_t Ciclos_sync = 0;
    ESP_LOGI(SNTP_TAG, "Inicializando sincronizacao SNTP...");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_SERVER);
    ESP_ERROR_CHECK(esp_netif_sntp_init(&config));

    setenv("TZ", "BRT3", 1);
    tzset();

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) == ESP_OK)
    {
        ESP_LOGI(SNTP_TAG, "Relogio sincronizado!");
    }
    else
    {
        ESP_LOGW(SNTP_TAG, "Sincronizacao inicial pendente.");
    }

    const TickType_t T = pdMS_TO_TICKS(TIME_T_MS);
    configASSERT(T > 0);
    TickType_t next = xTaskGetTickCount();

    for (;;)
    {
        uint64_t task_start_us = esp_timer_get_time();
        time_t now;
        struct tm timeinfo;
        char buffer[64];
        
        time(&now);
        localtime_r(&now, &timeinfo);
        
        if (timeinfo.tm_year + 1900 >= 2024)
        {
            strftime(buffer, sizeof(buffer), "%d/%m/%Y %H:%M:%S", &timeinfo);
            ESP_LOGI(SNTP_TAG, "Data e hora: %s", buffer);
            ESP_LOGI(SNTP_TAG, "Ciclos sincronizados %u", Ciclos_sync);
        }
        
        runs_time++;
        uint64_t task_end_us = esp_timer_get_time();
        misses_time += task_end_us - time_event_us > D_TIME_TASK * 1000LL
        vTaskDelayUntil(&next, T);
    }
}

// funcao usada em monitor_task
static const char *monitor_disparar_task(const char *cmd, int64_t received_us, int64_t received_ms)
{
    (void)received_ms;

    // Comando para FAIL-SAFE
    if (strcmp(cmd, "fs") == 0 || strcmp(cmd, "FS") == 0)
    {
        if (semFS == NULL)
            return "ERRO FS: SEMAFORO INVALIDO\n";

        fs_event_us = received_us;

        if (xSemaphoreGive(semFS) == pdTRUE)
        {
            ESP_LOGI("MONITOR", "FS_TASK acionada pelo PC");
            return "OK FS\n";
        }

        return "ERRO FS: SEMAFORO OCUPADO\n";
    }

    // Comando para NAVEGACAO
    if (strcmp(cmd, "nav") == 0 || strcmp(cmd, "NAV") == 0)
    {
        if (qNav == NULL)
            return "ERRO NAV: FILA INVALIDA\n";

        nav_evt_t evento = EV_NAV;
        nav_event_us = received_us;

        if (xQueueSend(qNav, &evento, 0) == pdTRUE)
        {
            ESP_LOGI("MONITOR", "NAV_PLAN acionada pelo PC");
            return "OK NAV\n";
        }

        return "ERRO NAV: FILA CHEIA\n";
    }

    return "COMANDO_INVALIDO\n";
}

// TASK_MONITOR 6 - Comunicacao TCP/UDP
static void task_monitor(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(MONITOR_T_MS);

    qMonitorLog = xQueueCreate(20, sizeof(monitor_log_t));

    int udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int tcp_server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int tcp_client = -1;

    struct sockaddr_in udp_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MONITOR_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    struct sockaddr_in tcp_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MONITOR_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    if (bind(udp, (struct sockaddr *)&udp_addr, sizeof(udp_addr)) < 0 ||
        bind(tcp_server, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0 ||
        listen(tcp_server, 1) < 0)
    {
        ESP_LOGE("MONITOR", "Erro ao iniciar sockets");
        close(udp);
        close(tcp_server);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in udp_peer = {0};
    bool udp_conectado = false;

    char buffer[MONITOR_BUFFER];
    char tcp_linha[MONITOR_BUFFER];
    size_t tcp_usados = 0;
    int maior;

    ESP_LOGI("MONITOR", "UDP porta %d | TCP porta %d", MONITOR_UDP_PORT, MONITOR_TCP_PORT);

    for (;;)
    {
        fd_set leitura;
        FD_ZERO(&leitura);

        FD_SET(udp, &leitura);
        FD_SET(tcp_server, &leitura);

        if (udp > tcp_server)
        {
            maior = udp;
        }
        else
        {
            maior = tcp_server;
        }

        if (tcp_client >= 0)
        {
            FD_SET(tcp_client, &leitura);
            if (tcp_client > maior)
                maior = tcp_client;
        }

        struct timeval timeout = {.tv_sec = 0, .tv_usec = 0};
        int prontos = select(maior + 1, &leitura, NULL, NULL, &timeout);

        if (prontos > 0)
        {
            // ================= UDP =================

            if (FD_ISSET(udp, &leitura))
            {
                struct sockaddr_in origem;
                socklen_t tamanho = sizeof(origem);

                int len = recvfrom(udp, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&origem, &tamanho);

                if (len > 0)
                {
                    buffer[len] = '\0';
                    buffer[strcspn(buffer, "\r\n")] = '\0';

                    udp_peer = origem;
                    udp_conectado = true;

                    ESP_LOGI("MONITOR", "UDP recebido: %s", buffer);

                    const char *resposta = monitor_disparar_task(buffer, esp_timer_get_time(), monitor_epoch_ms());

                    if (resposta == NULL)
                        resposta = "COMANDO_INVALIDO\n";

                    sendto(udp, resposta, strlen(resposta), 0, (struct sockaddr *)&origem, tamanho);

                    if (strncmp(resposta, "OK", 2) == 0)
                    {
                        monitor_log("Comando recebido via UDP");
                    }
                }
            }

            // ================= TCP - CONEXAO =================

            if (FD_ISSET(tcp_server, &leitura))
            {
                int novo_cliente = accept(tcp_server, NULL, NULL);

                if (novo_cliente >= 0)
                {
                    if (tcp_client < 0)
                    {
                        tcp_client = novo_cliente;
                        tcp_usados = 0;

                        struct timeval envio_timeout = {.tv_sec = 0, .tv_usec = 20000};
                        setsockopt(tcp_client, SOL_SOCKET, SO_SNDTIMEO, &envio_timeout, sizeof(envio_timeout));

                        ESP_LOGI("MONITOR", "Cliente TCP conectado");
                    }
                    else
                    {
                        close(novo_cliente);
                    }
                }
            }

            // ================= TCP - COMANDOS =================

            if (tcp_client >= 0 && FD_ISSET(tcp_client, &leitura))
            {
                int len = recv(tcp_client, buffer, sizeof(buffer), 0);

                if (len <= 0)
                {
                    close(tcp_client);
                    tcp_client = -1;
                    tcp_usados = 0;
                    ESP_LOGI("MONITOR", "Cliente TCP desconectado");
                }
                else
                {
                    for (int i = 0; i < len; i++)
                    {
                        if (buffer[i] == '\n')
                        {
                            tcp_linha[tcp_usados] = '\0';

                            ESP_LOGI("MONITOR", "TCP recebido: %s", tcp_linha);

                            const char *resposta = monitor_disparar_task(tcp_linha, esp_timer_get_time(), monitor_epoch_ms());

                            if (resposta == NULL)
                                resposta = "COMANDO_INVALIDO\n";

                            if (send(tcp_client, resposta, strlen(resposta), 0) < 0)
                            {
                                close(tcp_client);
                                tcp_client = -1;
                                tcp_usados = 0;
                                break;
                            }

                            if (strncmp(resposta, "OK", 2) == 0)
                            {
                                monitor_log("Comando recebido via TCP");
                            }

                            tcp_usados = 0;
                        }
                        else if (buffer[i] != '\r')
                        {
                            if (tcp_usados < sizeof(tcp_linha) - 1)
                            {
                                tcp_linha[tcp_usados++] = buffer[i];
                            }
                            else
                            {
                                tcp_usados = 0;
                            }
                        }
                    }
                }
            }
        }

        // ================= ENVIO DOS LOGS =================

        monitor_log_t evento;

        if (udp_conectado || tcp_client >= 0)
        {
            for (int i = 0; i < 2; i++)
            {
                if (xQueueReceive(qMonitorLog, &evento, 0) != pdTRUE)
                    break;

                if (udp_conectado)
                {
                    sendto(udp, evento.texto, strlen(evento.texto), 0, (struct sockaddr *)&udp_peer, sizeof(udp_peer));
                }

                if (tcp_client >= 0)
                {
                    if (send(tcp_client, evento.texto, strlen(evento.texto), 0) < 0)
                    {
                        close(tcp_client);
                        tcp_client = -1;
                        tcp_usados = 0;
                    }
                }
            }
        }

        vTaskDelayUntil(&next, T);
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

// =================== INICIALIZACAO ===================

// static void servidores_iniciar()
// {
//     BaseType_t udp_ok = xTaskCreate(udp_server_task, "UDP_SERVER", 4096, NULL, 5, NULL);
//     BaseType_t tcp_ok = xTaskCreate(tcp_server_task, "TCP_SERVER", 4096, NULL, 5, NULL);

//     configASSERT(udp_ok == pdPASS);
//     configASSERT(tcp_ok == pdPASS);
// }

// ======================================================
// APP MAIN
// ======================================================

void app_main(void)
{
    // IPC
    qNav = xQueueCreate(8, sizeof(nav_evt_t));
    semFS = xSemaphoreCreateBinary();

    // conectar wifi
    wifi_conectar();

    // criar tasks
    xTaskCreatePinnedToCore(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS, 0);
    xTaskCreatePinnedToCore(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL, 0);
    xTaskCreatePinnedToCore(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV, 0);
    xTaskCreatePinnedToCore(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS, 0);
    xTaskCreatePinnedToCore(task_monitor, "MONITOR", 8192, NULL, PRIO_MONITOR, NULL, 1);
    xTaskCreatePinnedToCore(task_time, "TIME", 4096, NULL, PRIO_TIME, NULL, 1);

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