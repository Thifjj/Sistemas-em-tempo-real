/* =====================================================================
   ETAPA 3 - INTERTAREFAS
   Duas tarefas periodicas, com um buffer compartilhado protegido por mutex:
     - SENSOR: a cada 10 ms escreve um texto novo (caracteres 1 a 255) no buffer
     - HUFFMAN: a cada 30 ms le o buffer e faz a codificacao (medida)
   Roda na ESP32 (ESP-IDF + FreeRTOS).

   Resposta de cada ativacao da Huffman = fim da ativacao - instante em que
   a tarefa acordou (liberada pelo vTaskDelay).
   ===================================================================== */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "huffman.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_cpu.h"
#include "esp_idf_version.h"
#include "esp_system.h"

#define NOME_ETAPA "Intertarefas"

/* Deadline D fixo para todas as etapas: D = HWM(100%) da etapa 1 x 1,2 = 11266 x 1,2 (us).
   Definido uma unica vez (plano, secao 4). Nao alterar entre etapas. */
#define DEADLINE_US 13519

#define MAX_ATIVACOES 5000 /* buffer estatico. 10000 estourou a DRAM em 32 KB */

#define PERIODO_SENSOR_MS 10  /* sensor escreve a cada 10 ms */
#define PERIODO_HUFFMAN_MS 30 /* huffman le e codifica a cada 30 ms */
#define PRIO_SENSOR 6
#define PRIO_HUFFMAN 5

typedef struct
{
    uint32_t tempos_us[MAX_ATIVACOES]; /* tempo de resposta de cada ativacao, em microssegundos */
    size_t quantidade;                 /* quantas ativacoes foram medidas */
    int erros;                         /* quantas validacoes falharam */
} Experimento;

static Experimento exp;

// Parametros usados pelas tarefas (variaveis estaticas, nao da pilha)
static size_t n_global;
static size_t tam_global;
static uint32_t semente_global;

// Buffer compartilhado entre sensor e huffman, protegido pelo mutex
static char *buffer;
static SemaphoreHandle_t mutex;
static SemaphoreHandle_t sem_fim; /* huffman libera o experimento quando termina */

// pegar a hora de agora sec+nanoseg (recebe ponteiro para gravar no original)
static void now(struct timespec *ts)
{
    clock_gettime(CLOCK_MONOTONIC, ts);
}

// diferenca entre dois instantes, em microssegundos
static double diferenca_us(struct timespec ini, struct timespec fim)
{
    return (double)(fim.tv_sec - ini.tv_sec) * 1e6 +
           (double)(fim.tv_nsec - ini.tv_nsec) / 1e3;
}

/* Sensor: escreve um texto novo no buffer, com caracteres aleatorios de 1 a 255
   (mesma carga de trabalho da etapa 1, para a comparacao ser justa). */
static void tarefa_sensor(void *arg)
{
    (void)arg;
    uint32_t estado = semente_global;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(PERIODO_SENSOR_MS));

        xSemaphoreTake(mutex, portMAX_DELAY);
        for (size_t j = 0; j < tam_global; j++)
        {
            /* mesma regra de huffman_gerar_texto (etapas 1 e 2): caracteres de 1 a 255 */
            buffer[j] = (char)(1 + xorshift32(&estado) % 255);
        }
        xSemaphoreGive(mutex);
    }
}

/* Huffman: periodica e medida. Copia o buffer (com mutex) e faz a codificacao. */
static void tarefa_huffman(void *arg)
{
    (void)arg;
    size_t n = n_global;
    size_t tam = tam_global;
    char *texto = (char *)malloc(tam + 1);

    for (size_t i = 0; i < n; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(PERIODO_HUFFMAN_MS));

        /* instante em que a tarefa foi liberada */
        struct timespec liberacao;
        now(&liberacao);

        /* copia o buffer compartilhado (esta copia ja conta no tempo da ativacao) */
        xSemaphoreTake(mutex, portMAX_DELAY);
        memcpy(texto, buffer, tam);
        xSemaphoreGive(mutex);
        texto[tam] = '\0';

        /* faz a Huffman: arvore -> codigos -> codificar -> decodificar */
        No *raiz = construir_arvore(texto);
        char **codigos = gerar_codigos(raiz);
        char *bits = codificar(texto, codigos);
        char *decodificado = decodificar(raiz, bits);

        /* mede: da liberacao ate o fim da Huffman */
        struct timespec fim;
        now(&fim);
        exp.tempos_us[i] = (uint32_t)diferenca_us(liberacao, fim);

        /* confere a saida */
        if (strcmp(texto, decodificado) != 0)
        {
            exp.erros++;
        }
        exp.quantidade++;

        /* libera a memoria desta ativacao (fora da medicao) */
        free(decodificado);
        free(bits);
        liberar_codigos(codigos);
        liberar_arvore(raiz);
    }

    free(texto);
    xSemaphoreGive(sem_fim);
    vTaskDelete(NULL);
}

/* Cria o sensor e a huffman, espera a huffman terminar e para o sensor */
static void executar_etapa3(size_t n, size_t tam, uint32_t semente)
{
    exp.quantidade = 0;
    exp.erros = 0;

    if (n > MAX_ATIVACOES)
    {
        n = MAX_ATIVACOES;
    }

    n_global = n;
    tam_global = tam;
    semente_global = semente;

    buffer = (char *)malloc(tam + 1);
    memset(buffer, 'a', tam);
    buffer[tam] = '\0';

    mutex = xSemaphoreCreateMutex();
    sem_fim = xSemaphoreCreateBinary();

    TaskHandle_t h_sensor;
    xTaskCreate(tarefa_sensor, "sensor", 3072, NULL, PRIO_SENSOR, &h_sensor);
    xTaskCreate(tarefa_huffman, "huffman", 8192, NULL, PRIO_HUFFMAN, NULL);

    xSemaphoreTake(sem_fim, portMAX_DELAY); /* espera a huffman acabar */

    vTaskDelete(h_sensor);
    vSemaphoreDelete(sem_fim);
    vSemaphoreDelete(mutex);
    free(buffer);
}

// Imprime as condicoes do teste que o proprio firmware consegue ler (plano, secao 3)
static void imprimir_condicoes(size_t n, size_t tamanho, uint32_t semente)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    printf("\n--- Condicoes do teste ---\n");
    printf("Etapa:            3 (%s)\n", NOME_ETAPA);
    printf("ESP-IDF:          %s\n", esp_get_idf_version());
    printf("Chip:             ESP32 (modelo %d), %d nucleos, revisao v%d.%d\n",
           (int)chip.model, (int)chip.cores, (int)(chip.revision / 100), (int)(chip.revision % 100));
    printf("CPU (config):     %d MHz\n", (int)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
#ifdef CONFIG_PM_ENABLE
    printf("Power management: ligado (CONFIG_PM_ENABLE)\n");
#else
    printf("Power management: desligado (frequencia fixa, equivale ao governor)\n");
#endif
#ifdef CONFIG_COMPILER_OPTIMIZATION_PERF
    printf("Otimizacao:       -O2 (CONFIG_COMPILER_OPTIMIZATION_PERF)\n");
#else
    printf("Otimizacao:       nao e -O2 (verificar sdkconfig)\n");
#endif
    printf("Tarefas (FreeRTOS): %u\n", (unsigned)uxTaskGetNumberOfTasks());
    printf("Nucleo da tarefa: %d\n", (int)esp_cpu_get_core_id());
#ifdef CONFIG_ESP_WIFI_ENABLED
    printf("Wi-Fi (componente): compilado\n");
#else
    printf("Wi-Fi (componente): nao compilado\n");
#endif
    printf("Periodos:         sensor %d ms, huffman %d ms\n", PERIODO_SENSOR_MS, PERIODO_HUFFMAN_MS);
    printf("Prioridades:      sensor %d, huffman %d\n", PRIO_SENSOR, PRIO_HUFFMAN);
    printf("Parametros:       n=%u, tamanho=%u, semente=%u\n",
           (unsigned)n, (unsigned)tamanho, (unsigned)semente);
    printf("--------------------------\n");
}

/* Calcula e imprime as perdas de deadline (plano, secao 6.3 e 6.4).
   Perda = tempo de resposta > DEADLINE_US. Roda depois da medicao. */
static void imprimir_perdas(const Experimento *e)
{
    size_t q = e->quantidade;
    size_t perdas = 0;
    size_t ultima = 0;            /* ativacao (1-based) da ultima perda */
    size_t menor_dist = SIZE_MAX; /* skip factor S */
    size_t rajada = 0, maior_rajada = 0;
    size_t prev = 0; /* ativacao (1-based) da perda anterior */

    printf("\n--- Perdas de deadline (D = %d us) ---\n", DEADLINE_US);
    printf("Ativacoes perdidas:");

    for (size_t i = 0; i < q; i++)
    {
        if (e->tempos_us[i] > DEADLINE_US)
        {
            size_t at = i + 1;
            printf(" %u", (unsigned)at);
            perdas++;

            if (perdas > 1)
            {
                size_t dist = at - ultima;
                if (dist < menor_dist)
                    menor_dist = dist;
            }
            /* rajada: perdas em ativacoes consecutivas */
            if (perdas > 1 && at == prev + 1)
                rajada = rajada + 1;
            else
                rajada = 1;
            if (rajada > maior_rajada)
                maior_rajada = rajada;

            ultima = at;
            prev = at;
        }
    }
    if (perdas == 0)
        printf(" nenhuma");
    printf("\n");

    double percentual = 0.0;
    if (q > 0)
        percentual = 100.0 * (double)(q - perdas) / (double)q;
    printf("Cumpriram o prazo: %u de %u (%.2f%%)\n",
           (unsigned)(q - perdas), (unsigned)q, percentual);
    printf("Perdas:            %u\n", (unsigned)perdas);
    if (perdas >= 2)
        printf("Skip factor S:     %u\n", (unsigned)menor_dist);
    else
        printf("Skip factor S:     n/a (menos de 2 perdas)\n");
    printf("Maior rajada:      %u\n", (unsigned)maior_rajada);

    /* (m,k)-firme: pior janela com mais perdas -> m = k - perdas_max */
    size_t ks[3] = {10, 20, 50};
    for (int j = 0; j < 3; j++)
    {
        size_t k = ks[j];
        if (q >= k)
        {
            size_t pior = 0, na_janela = 0;
            for (size_t i = 0; i < q; i++)
            {
                if (e->tempos_us[i] > DEADLINE_US)
                    na_janela++;
                if (i >= k && e->tempos_us[i - k] > DEADLINE_US)
                    na_janela--;
                if (i + 1 >= k && na_janela > pior)
                    pior = na_janela;
            }
            printf("(m,k) k=%u: pior janela com %u perdas -> (%u,%u)\n",
                   (unsigned)k, (unsigned)pior, (unsigned)(k - pior), (unsigned)k);
        }
    }
    printf("----------------------------------------\n");
}

// Imprime o resumo estatistico: min, media e HWM do tempo de resposta
static void imprimir_resumo(size_t tamanho, uint32_t semente)
{
    if (exp.quantidade == 0)
    {
        printf("Nenhuma ativacao medida.\n");
        return;
    }

    uint64_t soma_total = 0;
    uint32_t menor = UINT32_MAX, maior = 0;

    for (size_t i = 0; i < exp.quantidade; i++)
    {
        uint32_t t = exp.tempos_us[i];
        soma_total += t;
        if (t < menor)
            menor = t;
        if (t > maior)
            maior = t; /* HWM do total */

        /* HWM parcial: mostra como o maximo se estabiliza */
        if ((i + 1) % 100 == 0)
            printf("Ativacao %4zu: HWM parcial = %u us\n", i + 1, (unsigned)maior);
    }

    double q = (double)exp.quantidade;

    printf("\n--- Experimento ETAPA 3 (%s): %zu ativacoes, array de %zu caracteres, semente %u ---\n",
           NOME_ETAPA, exp.quantidade, tamanho, (unsigned)semente);
    printf("Erros:            %d\n", exp.erros);
    printf("Total (us):       min %u  media %.1f  HWM %u\n", (unsigned)menor, (double)soma_total / q, (unsigned)maior);
}

static void rodar_experimento(void)
{
    size_t n = 1000;          /* quantas ativacoes da huffman */
    size_t tamanho = 1000;    /* tamanho do texto lido do buffer (caracteres) */
    uint32_t semente = 12345; /* semente do sensor, registrada no relatorio */

    imprimir_condicoes(n, tamanho, semente);

    executar_etapa3(n, tamanho, semente);

    imprimir_resumo(tamanho, semente);
    imprimir_perdas(&exp);
}

// Tarefa sem core fixo (o scheduler escolhe o core), executa o experimento uma vez
static void tarefa_experimento(void *arg)
{
    (void)arg;
    rodar_experimento();
    vTaskDelete(NULL);
}

// Ponto de entrada na ESP32 (ESP-IDF)
void app_main(void)
{
    xTaskCreate(tarefa_experimento, "experimento", 8192, NULL, 5, NULL);
}
