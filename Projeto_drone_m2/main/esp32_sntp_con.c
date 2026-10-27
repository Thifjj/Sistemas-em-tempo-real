/*
Arquivo: esp32_sntp_con.c
Autor: Felipe Viel
Disciplina: Sistemas em Tempo Real
Engenharia de Computação - Univali
Descrição: Código que permite a ESP32 capturar a hora e data atual em servidor SNTP
Garanta que no CMakeList esteja a configuração abaixo:

idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_wifi nvs_flash
                       INCLUDE_DIRS "")
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include <time.h>

static const char *TAG = "TIME";

#define WIFI_SSID "Nome da Rede WiFi"
#define WIFI_PASS "Senha WiFi"

//Configura WiFi da ESP32
static void wifi_init(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wc = {0};
    strcpy((char*)wc.sta.ssid, WIFI_SSID);
    strcpy((char*)wc.sta.password, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

//Apenas para Log
static void time_sync_notification_cb(struct timeval *tv){
    ESP_LOGI(TAG, "Tempo sincronizado via SNTP.");
}

void app_main(void){
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

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

    // Aguarda 1ª sincronização (até 15 s)
    for (int i=0; i<30 && sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET; ++i) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGW(TAG, "Ainda sem sync. Verifique WiFi/NTP.");
    } else {
        char buf[64];
        strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S %Z", &timeinfo);
        ESP_LOGI(TAG, "Hora atual: %s", buf);
    }

    // Loop de demonstração
    while (1) {
        time(&now);
        localtime_r(&now, &timeinfo);
        char buf[64];
        strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
        ESP_LOGI(TAG, "%s", buf);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}