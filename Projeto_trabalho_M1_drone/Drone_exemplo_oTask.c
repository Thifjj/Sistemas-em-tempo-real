// Drone (ESP32 + FreeRTOS) — 4 tasks com touch ISR
// - FUS_IMU (periódica, 5 ms) -> notifica CTRL_ATT
// - CTRL_ATT (hard RT) -> roda após FUS_IMU
// - NAV_PLAN (soft RT) -> eventos por touch B/C
// - FS_TASK (hard RT) -> emergência por touch D (ISR)
// Compilado com ESP-IDF 5.x

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h" /
/ API oficial de touch pad : contentReference [oaicite:1]{index = 1}

#define TAG "DRONE"

// ====== Mapeamento dos touch pads ======
// Escolha pinos que não atrapalhem o boot: T7=GPIO27, T8=GPIO33, T9=GPIO32
#define TP_NAV TOUCH_PAD_NUM7 // Touch B -> NAV
#define TP_TEL TOUCH_PAD_NUM8 // Touch C -> Telemetria
#define TP_FS TOUCH_PAD_NUM9  // Touch D -> Fail-safe

// ====== Periodicidade e prioridades ======
#define FUS_T_MS 5
// Exemplo de prioridades (critério "custom": segurança no topo)
#define PRIO_FS_TASK 5
#define PRIO_FUS_IMU 4
#define PRIO_CTRL_ATT 3
#define PRIO_NAV_PLAN 2
#define STK 3072

                             typedef struct
{
    float m1;
    float m2;
    float m3;
    float m4;
} esc_output_t;

static esc_output_t g_esc = {0};

// ====== Comuns/IPC ======
static TaskHandle_t hFUS = NULL, hCTRL = NULL, hNAV = NULL, hFS = NULL;

// Evento para NAV/Telemetria
typedef enum
{
    EV_NAV = 1,
    EV_TEL = 2
} nav_evt_t;
static QueueHandle_t qNav = NULL;

// Sinal para Fail-safe
static SemaphoreHandle_t semFS = NULL;

// Estado "simulado"
typedef struct
{
    float roll, pitch, yaw; // deg
} state_t;

static state_t g_state = {0};

static float clamp(float value, float min, float max)
{
    if (value < min)
        return min;

    if (value > max)
        return max;

    return value;
}

static void esc_write_simulated(
    float m1,
    float m2,
    float m3,
    float m4)
{
    g_esc.m1 = clamp(m1, 0.0f, 100.0f);
    g_esc.m2 = clamp(m2, 0.0f, 100.0f);
    g_esc.m3 = clamp(m3, 0.0f, 100.0f);
    g_esc.m4 = clamp(m4, 0.0f, 100.0f);
}

// ====== Util: "busy" previsível (~C_exec) ======
static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us)
    {
        __asm__ __volatile__("nop");
    }
}

// ====== Fusão (periódica 5ms) ======
static void task_fus_imu(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(FUS_T_MS);

    for (;;)
    {
        // Simulação rápida de filtro (Madgwick/Complementar "fake"):
        // Atualiza estado com uma dinâmica qualquer e executa "carga" ~1.0 ms
        g_state.roll *= 0.98f;
        g_state.pitch *= 0.98f;
        g_state.yaw *= 0.98f;
        g_state.roll += 0.1f;
        g_state.yaw += 0.05f;

        cpu_tight_loop_us(1000); // ~1.0 ms (WCET aproximado)

        // Notifica o controle
        if (hCTRL)
            xTaskNotifyGive(hCTRL);

        // Cumprir periodicidade determinística
        vTaskDelayUntil(&next, T);
    }
}

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

// ====== Navegação/Telemetria (soft RT), acionada por touch B/C ======
static void task_nav_plan(void *arg)
{
    nav_evt_t ev;
    for (;;)
    {
        if (xQueueReceive(qNav, &ev, portMAX_DELAY) == pdTRUE)
        {
            if (ev == EV_NAV)
            {
                // Atualiza waypoint/plano simples (~3-4 ms)
                cpu_tight_loop_us(3500);
            }
            else if (ev == EV_TEL)
            {
                // Telemetria leve: imprime estado (não bloquear muito)
                printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f", g_state.roll, g_state.pitch, g_state.yaw);
                cpu_tight_loop_us(500);
            }
        }
    }
}

// ====== Fail-safe (hard RT) acionado por touch D (ISR -> semáforo) ======
static void task_fail_safe(void *arg)
{
    for (;;)
    {
        if (xSemaphoreTake(semFS, portMAX_DELAY) == pdTRUE)
        {
            int64_t t0 = esp_timer_get_time();
            // Ação crítica: reduzir "throttle", travar modos, etc. (~0.8-1.0 ms)
            cpu_tight_loop_us(900);
            int64_t dt = esp_timer_get_time() - t0;
            printf("FAIL-SAFE! tratado em %lld us", (long long)dt);
        }
    }
}

// ====== app_main ======
void app_main(void)
{
    // IPC
    qNav = xQueueCreate(8, sizeof(nav_evt_t));
    semFS = xSemaphoreCreateBinary();

    // Cria tasks
    xTaskCreatePinnedToCore(task_fail_safe, "FS_TASK", STK, NULL, PRIO_FS_TASK, &hFS, 0);
    xTaskCreatePinnedToCore(task_fus_imu, "FUS_IMU", STK, NULL, PRIO_FUS_IMU, &hFUS, 0);
    xTaskCreatePinnedToCore(task_ctrl_att, "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL, 0);
    xTaskCreatePinnedToCore(task_nav_plan, "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV, 0);
}
