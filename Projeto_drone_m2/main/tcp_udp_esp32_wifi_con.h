// Wi-Fi e sockets usados pelo monitor. O processamento de rede nao bloqueia.
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *NET_TAG = "REDE";
#define WIFI_SSID "Jacques"
#define WIFI_PASS "74336456"
#define TCP_PORT 5000 // qualquer número entre 0 e 65535

#define PC_IP   "172.16.0.14"   // IP do PC - use ifconfig (Linux) ou ipconfig (Windows) para achar o IP
#define PC_PORT 6010 // qualquer número entre 0 e 65535

// O bit indica que a interface recebeu um endereco IP valido.
#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_events;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = data;
        xEventGroupClearBits(wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGW(NET_TAG, "Wi-Fi desconectado (motivo=%u); tentando reconectar",
                 (unsigned)event->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;
        ESP_LOGI(NET_TAG, "IP obtido: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

// Inicializacao unica; o callback acompanha conexoes e desconexoes posteriores.
static void wifi_init(void)
{
    wifi_events = xEventGroupCreate();
    assert(wifi_events != NULL);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    wifi_config_t wc = { .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK } };
    strlcpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}
