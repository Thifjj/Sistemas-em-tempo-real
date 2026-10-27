/*
Arquivo: tcp_udp_esp32_wifi_con.c
Autor: Felipe Viel
Disciplina: Sistemas em Tempo Real
Engenharia de Computação - Univali
Descrição: Código que torna a ESP32 Cliente (TCP) ou Servidor (UDP) na troca de pacotes com um computador
Garanta que no CMakeList esteja a configuração abaixo:

idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_wifi nvs_flash
                       INCLUDE_DIRS "")
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "TCP_SERVER";
#define WIFI_SSID "Nome da sua rede WiFi"
#define WIFI_PASS "senha do WiFi"
#define TCP_PORT 5000 // qualquer número entre 0 e 65535

#define PC_IP   "10.27.142.157"   // IP do PC - use ifconfig (Linux) ou ipconfig (Windows) para achar o IP
#define PC_PORT 6010 // qualquer número entre 0 e 65535

//Gera dados e envia para o Servidor UDP no computador
static void udp_task(void *arg){
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(PC_PORT);
    inet_pton(AF_INET, PC_IP, &dest.sin_addr.s_addr);

    while (1) {
        char payload[64];
        static int seq = 0;
        snprintf(payload, sizeof(payload), "{\"seq\":%d,\"ax\":0.12,\"ay\":-0.03}", seq++);
        sendto(sock, payload, strlen(payload), 0, (struct sockaddr*)&dest, sizeof(dest));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

//Configura WiFi da ESP32
static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Conectando ao WiFi...");
    ESP_ERROR_CHECK(esp_wifi_connect());
}

//Espera pacotes enviados pelo PC usando Protocolo TCP e devolve informações
static void tcp_server_task(void *arg) {
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr));
    listen(listen_fd, 1);
    ESP_LOGI(TAG, "Servidor TCP na porta %d", TCP_PORT);

    while (1) {
        struct sockaddr_in6 source_addr; socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_fd, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) continue;
        ESP_LOGI(TAG, "Cliente conectado");

        // Envia mensagem de boas-vindas
        const char *hello = "ESP32: conectado!\n";
        send(sock, hello, strlen(hello), 0);

        char rx[256];
        while (1) {
            int len = recv(sock, rx, sizeof(rx)-1, 0);
            if (len <= 0) { ESP_LOGI(TAG, "Cliente saiu"); break; }
            rx[len] = 0;
            ESP_LOGI(TAG, "RX: %s", rx);

            // Eco + resposta JSON simples
            char tx[300];
            snprintf(tx, sizeof(tx), "{\"ok\":true,\"echo\":\"%s\"}\n", rx);
            send(sock, tx, strlen(tx), 0);
        }
        shutdown(sock, 0);
        close(sock);
    }
}


void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    //xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(udp_task, "udp_task", 4096, NULL, 5, NULL);
}