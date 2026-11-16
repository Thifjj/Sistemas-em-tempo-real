#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "huffman.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define MAX_ATIVACOES 10000 /* tamanho do buffer: 10000 * 4 bytes = 40 KB */

// Funcoes e estruturas para realizar o experimento
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
    Tempos etapas[MAX_ATIVACOES];      /* tempo de cada etapa, por ativacao */
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

/* Roda n ativacoes do mesmo ciclo do main (arvore -> codigos -> codificar ->
   decodificar -> conferir), medindo cada etapa e o total.
   Cada ativacao usa uma entrada nova (semente_base + i). Nao imprime nada. */
void executar_experimento(Experimento *e, size_t n, size_t tam_texto, uint32_t semente_base)
{
    e->quantidade = 0;
    e->erros = 0;

    if (n > MAX_ATIVACOES)
    {
        n = MAX_ATIVACOES; /* nao estoura o buffer */
    }

    char *texto = (char *)malloc(tam_texto + 1);
    if (texto == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        exit(1);
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
        e->etapas[i].criar_arvore_us = (uint32_t)diferenca_us(s, f);

        /* 2) gera os codigos */
        now(&s);
        char **codigos = gerar_codigos(raiz);
        now(&f);
        e->etapas[i].gerar_codigos_us = (uint32_t)diferenca_us(s, f);

        /* 3) codifica */
        now(&s);
        char *bits = codificar(texto, codigos);
        now(&f);
        e->etapas[i].codificar_us = (uint32_t)diferenca_us(s, f);

        /* 4) decodifica */
        now(&s);
        char *decodificado = decodificar(raiz, bits);
        now(&f);
        e->etapas[i].decodificar_us = (uint32_t)diferenca_us(s, f);

        now(&fim_total);

        e->tempos_us[i] = (uint32_t)diferenca_us(ini_total, fim_total);

        /* 5) conferencia */
        if (strcmp(texto, decodificado) != 0)
        {
            e->erros++;
        }
        e->quantidade++;

        /* libera tudo desta ativacao (fora da medicao) */
        free(decodificado);
        free(bits);
        liberar_codigos(codigos);
        liberar_arvore(raiz);
    }

    free(texto);
}

// Roda o experimento e imprime o resultado. Retorna 0 se nao houve erro.
static int rodar_experimento(void)
{
    size_t n = 1000;          /* quantas ativacoes no experimento */
    size_t tamanho = 1000;    /* tamanho do array aleatorio (caracteres) */
    uint32_t semente = 12345; /* semente base, registrada no relatorio */

    /* experimento: n ativacoes medidas, cada uma com um array aleatorio
       de 'tamanho' caracteres (sem impressao durante a medicao) */
    executar_experimento(&exp, n, tamanho, semente);

    uint64_t soma_total = 0;
    uint32_t menor = UINT32_MAX, maior = 0;
    uint64_t soma_criar = 0, soma_gerar = 0, soma_codificar = 0, soma_decodificar = 0;

    /* HWM (maior tempo ate agora) de cada etapa e do total */
    uint32_t hwm_criar = 0, hwm_gerar = 0, hwm_codificar = 0, hwm_decodificar = 0;

    for (size_t i = 0; i < exp.quantidade; i++)
    {
        uint32_t t = exp.tempos_us[i];
        soma_total += t;
        if (t < menor)
            menor = t;
        if (t > maior)
            maior = t; /* HWM do total */

        soma_criar += exp.etapas[i].criar_arvore_us;
        soma_gerar += exp.etapas[i].gerar_codigos_us;
        soma_codificar += exp.etapas[i].codificar_us;
        soma_decodificar += exp.etapas[i].decodificar_us;

        // HWM de cada etapa
        if (exp.etapas[i].criar_arvore_us > hwm_criar)
            hwm_criar = exp.etapas[i].criar_arvore_us;
        if (exp.etapas[i].gerar_codigos_us > hwm_gerar)
            hwm_gerar = exp.etapas[i].gerar_codigos_us;
        if (exp.etapas[i].codificar_us > hwm_codificar)
            hwm_codificar = exp.etapas[i].codificar_us;
        if (exp.etapas[i].decodificar_us > hwm_decodificar)
            hwm_decodificar = exp.etapas[i].decodificar_us;

        /* HWM parcial: mostra como o maximo se estabiliza */
        if ((i + 1) % 100 == 0)
            printf("Ativacao %4zu: HWM parcial = %u us\n", i + 1, (unsigned)maior);
    }

    double q = (double)exp.quantidade;

    printf("\n--- Experimento (%zu ativacoes, array de %zu caracteres, semente %u) ---\n",
           exp.quantidade, tamanho, (unsigned)semente);
    printf("Erros:            %d\n", exp.erros);
    printf("Total (us):       min %u  media %.1f  HWM %u\n", (unsigned)menor, (double)soma_total / q, (unsigned)maior);
    printf("Criar arvore:     media %.1f  HWM %u us\n", (double)soma_criar / q, (unsigned)hwm_criar);
    printf("Gerar codigos:    media %.1f  HWM %u us\n", (double)soma_gerar / q, (unsigned)hwm_gerar);
    printf("Codificar:        media %.1f  HWM %u us\n", (double)soma_codificar / q, (unsigned)hwm_codificar);
    printf("Decodificar:      media %.1f  HWM %u us\n", (double)soma_decodificar / q, (unsigned)hwm_decodificar);

    return exp.erros == 0 ? 0 : 1;
}

#ifdef ESP_PLATFORM
// Tarefa fixada no core 1 (app core), que executa o experimento uma vez
static void tarefa_experimento(void *arg)
{
    (void)arg;
    rodar_experimento();
    vTaskDelete(NULL);
}

// Ponto de entrada na ESP32 (ESP-IDF)
void app_main(void)
{
    xTaskCreatePinnedToCore(tarefa_experimento, "experimento", 8192, NULL, 5, NULL, 1);
}
#else
// Ponto de entrada no PC (Linux)
int main(void)
{
    return rodar_experimento();
}
#endif
