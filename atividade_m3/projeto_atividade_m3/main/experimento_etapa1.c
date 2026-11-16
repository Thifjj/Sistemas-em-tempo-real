/* =====================================================================
   ETAPA 1 - TAREFA
   Huffman isolada: n ativacoes em sequencia, sem evento e sem estresse.
   Roda na ESP32 (ESP-IDF + FreeRTOS).
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
#include "esp_chip_info.h"
#include "esp_cpu.h"
#include "esp_idf_version.h"
#include "esp_system.h"

#define NOME_ETAPA "Tarefa"

/* Deadline D fixo para todas as etapas: D = HWM(100%) da etapa 1 x 1,2 = 11266 x 1,2 (us).
   Definido uma unica vez (plano, secao 4). Nao alterar entre etapas. */
#define DEADLINE_US 13519

#define MAX_ATIVACOES 5000 /* buffer estatico: tempos (20 KB) + etapas (80 KB). 10000 estourou a DRAM em 32 KB */

// Estruturas para guardar os tempos
typedef struct Tempos
{
    uint32_t criar_arvore_us;
    uint32_t gerar_codigos_us;
    uint32_t codificar_us;
    uint32_t decodificar_us;
} Tempos;

typedef struct
{
    uint32_t tempos_us[MAX_ATIVACOES]; /* tempo total de cada ativacao, em microssegundos */
    Tempos etapas[MAX_ATIVACOES];      /* tempo de cada etapa interna da Huffman, por ativacao */
    size_t quantidade;                 /* quantas ativacoes foram medidas */
    int erros;                         /* quantas validacoes falharam */
} Experimento;

static Experimento exp;

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

// Imprime as condicoes do teste que o proprio firmware consegue ler (plano, secao 3)
static void imprimir_condicoes(size_t n, size_t tamanho, uint32_t semente)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    printf("\n--- Condicoes do teste ---\n");
    printf("Etapa:            1 (%s)\n", NOME_ETAPA);
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
    printf("Parametros:       n=%u, tamanho=%u, semente=%u\n",
           (unsigned)n, (unsigned)tamanho, (unsigned)semente);
    printf("--------------------------\n");
}

/* Roda n ativacoes (arvore -> codigos -> codificar -> decodificar -> conferir),
   medindo cada etapa interna e o total. Cada ativacao usa uma entrada nova
   (semente_base + i). Nao imprime nada durante a medicao. */
static void executar_etapa1(size_t n, size_t tam_texto, uint32_t semente_base)
{
    exp.quantidade = 0;
    exp.erros = 0;

    if (n > MAX_ATIVACOES)
    {
        n = MAX_ATIVACOES; /* nao estoura o buffer */
    }

    char *texto = (char *)malloc(tam_texto + 1);
    if (texto == NULL)
    {
        printf("Erro: sem memoria\n");
        return;
    }

    struct timespec s, f, ini_total, fim_total;

    for (size_t i = 0; i < n; i++)
    {
        /* entrada nova para esta ativacao (fora da medicao) */
        huffman_gerar_texto(texto, tam_texto, semente_base + (uint32_t)i);

        now(&ini_total);

        /* 1) monta a arvore */
        now(&s);
        No *raiz = construir_arvore(texto);
        now(&f);
        exp.etapas[i].criar_arvore_us = (uint32_t)diferenca_us(s, f);

        /* 2) gera os codigos */
        now(&s);
        char **codigos = gerar_codigos(raiz);
        now(&f);
        exp.etapas[i].gerar_codigos_us = (uint32_t)diferenca_us(s, f);

        /* 3) codifica */
        now(&s);
        char *bits = codificar(texto, codigos);
        now(&f);
        exp.etapas[i].codificar_us = (uint32_t)diferenca_us(s, f);

        /* 4) decodifica */
        now(&s);
        char *decodificado = decodificar(raiz, bits);
        now(&f);
        exp.etapas[i].decodificar_us = (uint32_t)diferenca_us(s, f);

        now(&fim_total);

        exp.tempos_us[i] = (uint32_t)diferenca_us(ini_total, fim_total);

        /* 5) conferencia */
        if (strcmp(texto, decodificado) != 0)
        {
            exp.erros++;
        }
        exp.quantidade++;

        /* libera tudo desta ativacao (fora da medicao) */
        free(decodificado);
        free(bits);
        liberar_codigos(codigos);
        liberar_arvore(raiz);

        vTaskDelay(1); /* bloqueia a tarefa por 1 tick para a task IDLE rodar e alimentar o watchdog (evita o task_wdt) */
    }

    free(texto);
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

// Imprime o resumo estatistico: min, media, HWM do total e das etapas internas
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

    printf("\n--- Experimento ETAPA 1 (%s): %zu ativacoes, array de %zu caracteres, semente %u ---\n",
           NOME_ETAPA, exp.quantidade, tamanho, (unsigned)semente);
    printf("Erros:            %d\n", exp.erros);
    printf("Total (us):       min %u  media %.1f  HWM %u\n", (unsigned)menor, (double)soma_total / q, (unsigned)maior);

    uint64_t soma_criar = 0, soma_gerar = 0, soma_codificar = 0, soma_decodificar = 0;
    uint32_t hwm_criar = 0, hwm_gerar = 0, hwm_codificar = 0, hwm_decodificar = 0;
    for (size_t i = 0; i < exp.quantidade; i++)
    {
        soma_criar += exp.etapas[i].criar_arvore_us;
        soma_gerar += exp.etapas[i].gerar_codigos_us;
        soma_codificar += exp.etapas[i].codificar_us;
        soma_decodificar += exp.etapas[i].decodificar_us;

        if (exp.etapas[i].criar_arvore_us > hwm_criar)
            hwm_criar = exp.etapas[i].criar_arvore_us;
        if (exp.etapas[i].gerar_codigos_us > hwm_gerar)
            hwm_gerar = exp.etapas[i].gerar_codigos_us;
        if (exp.etapas[i].codificar_us > hwm_codificar)
            hwm_codificar = exp.etapas[i].codificar_us;
        if (exp.etapas[i].decodificar_us > hwm_decodificar)
            hwm_decodificar = exp.etapas[i].decodificar_us;
    }
    printf("Criar arvore:     media %.1f  HWM %u us\n", (double)soma_criar / q, (unsigned)hwm_criar);
    printf("Gerar codigos:    media %.1f  HWM %u us\n", (double)soma_gerar / q, (unsigned)hwm_gerar);
    printf("Codificar:        media %.1f  HWM %u us\n", (double)soma_codificar / q, (unsigned)hwm_codificar);
    printf("Decodificar:      media %.1f  HWM %u us\n", (double)soma_decodificar / q, (unsigned)hwm_decodificar);
}

static void rodar_experimento(void)
{
    size_t n = 1000;          /* quantas ativacoes no experimento */
    size_t tamanho = 1000;    /* tamanho do array aleatorio (caracteres) */
    uint32_t semente = 12345; /* semente base, registrada no relatorio */

    imprimir_condicoes(n, tamanho, semente);

    executar_etapa1(n, tamanho, semente);

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
