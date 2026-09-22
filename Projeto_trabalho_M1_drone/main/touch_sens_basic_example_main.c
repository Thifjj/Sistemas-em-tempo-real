/*
 * Drone ESP32 + FreeRTOS
 * ESP-IDF 6.1
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

#define TAG "DRONE"

// ======================================================
// PERIODICIDADE E PRIORIDADES
// ======================================================

#define FUS_T_MS 5

#define PRIO_FS_TASK 5
#define PRIO_FUS_IMU 4
#define PRIO_CTRL_ATT 3
#define PRIO_NAV_PLAN 2

#define STK 3072

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

static void task_fus_imu(void *arg)
{
    TickType_t next = xTaskGetTickCount();

    const TickType_t T = pdMS_TO_TICKS(FUS_T_MS);

    for (;;)
    {

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
            xTaskNotifyGive(hCTRL);
        }

        // Mantém período de 5 ms
        vTaskDelayUntil(&next, T);
    }
}

// ======================================================
// TASK 2 - CONTROLE DE ATITUDE
// ======================================================

// ====== Controle de atitude (espera notificação da FUS_IMU) ======
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
    }
}

// ======================================================
// TASK 3 - NAVEGAÇÃO / TELEMETRIA
// ======================================================

static void task_nav_plan(void *arg)
{
    nav_evt_t ev;
    for (;;)
    {
        // Fica bloqueada esperando algo entrar na Queue
        if (
            xQueueReceive(qNav, &ev, portMAX_DELAY) == pdTRUE)
        {
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
        }
    }
}

// ======================================================
// TASK 4 - FAIL SAFE
// ======================================================

static void task_fail_safe(void *arg)
{
    for (;;)
    {

        // Fica bloqueada esperando o semáforo
        if (xSemaphoreTake(semFS, portMAX_DELAY) == pdTRUE)
        {
            int64_t t0 = esp_timer_get_time();

            // Simula ação crítica:
            // reduzir throttle / hover / pouso

            cpu_tight_loop_us(900);

            int64_t dt = esp_timer_get_time() - t0;

            printf("FAIL-SAFE! tratado em %lld us\n", (long long)dt);
        }
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
    3
};

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

        xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken);

        break;
    }

    case 9:
    {
        nav_evt_t ev = EV_TEL;
        xQueueSendFromISR(qNav, &ev, &higher_priority_task_woken);

        break;
    }

    case 0:
    {
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
    // IPC
    qNav = xQueueCreate(8, sizeof(nav_evt_t));
    semFS = xSemaphoreCreateBinary();

    // Cria tasks
    xTaskCreatePinnedToCore(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS, 0);
    xTaskCreatePinnedToCore(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL, 0);
    xTaskCreatePinnedToCore(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV, 0);
    xTaskCreatePinnedToCore(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS, 0);

    ESP_ERROR_CHECK(example_touch_init());

    ESP_LOGI(TAG, "Sistema iniciado");

    while (1)
    {

        vTaskDelay(
            pdMS_TO_TICKS(1000));
    }
}