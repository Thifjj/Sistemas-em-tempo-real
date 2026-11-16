#ifndef TCP_UDP_SERVER_H
#define TCP_UDP_SERVER_H

#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#define UDP_PORT 6010
#define TCP_PORT 6011
#define BUFFER_SIZE 1024

static const char *SERVER_TAG = "SERVER";

// =================== UDP ===================

static void udp_server_task(void *pvParameters)
{
    char buffer[BUFFER_SIZE];

    while (1) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        if (sock < 0) {
            ESP_LOGE(SERVER_TAG, "Erro ao criar socket UDP");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in server_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(UDP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY)
        };

        if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            ESP_LOGE(SERVER_TAG, "Erro no bind UDP");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(SERVER_TAG, "UDP aguardando na porta %d", UDP_PORT);

        while (1) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);

            int len = recvfrom(sock, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *)&client_addr, &addr_len);

            if (len < 0) break;

            buffer[len] = '\0';

            ESP_LOGI(SERVER_TAG, "[UDP] Recebido: %s", buffer);

            const char *resposta = "ESP32 recebeu via UDP";
            sendto(sock, resposta, strlen(resposta), 0, (struct sockaddr *)&client_addr, addr_len);
        }

        close(sock);
    }
}

// =================== TCP ===================

static void tcp_server_task(void *pvParameters)
{
    char buffer[BUFFER_SIZE];

    while (1) {
        int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

        if (server_sock < 0) {
            ESP_LOGE(SERVER_TAG, "Erro ao criar socket TCP");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in server_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY)
        };

        if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0 || listen(server_sock, 1) < 0) {
            ESP_LOGE(SERVER_TAG, "Erro ao iniciar TCP");
            close(server_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(SERVER_TAG, "TCP aguardando na porta %d", TCP_PORT);

        while (1) {
            int client_sock = accept(server_sock, NULL, NULL);

            if (client_sock < 0) break;

            ESP_LOGI(SERVER_TAG, "Cliente TCP conectado");

            while (1) {
                int len = recv(client_sock, buffer, BUFFER_SIZE - 1, 0);

                if (len <= 0) break;

                buffer[len] = '\0';

                ESP_LOGI(SERVER_TAG, "[TCP] Recebido: %s", buffer);

                const char *resposta = "ESP32 recebeu via TCP";
                if (send(client_sock, resposta, strlen(resposta), 0) < 0) break;
            }

            close(client_sock);
            ESP_LOGI(SERVER_TAG, "Cliente TCP desconectado");
        }

        close(server_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif