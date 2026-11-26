// periodic_server_example.c
// gcc -O2 -Wall servidor_aperiodico.c -o servidor_aperiodico -pthread
// sudo ./servidor_aperiodico          (carga simulada com nanosleep)
// sudo ./servidor_aperiodico nop      (carga simulada com laco de NOP)
#define _GNU_SOURCE
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#define NS_PER_SEC 1000000000L
#define NS_PER_MS 1000000L

/////////////////////// FILA DE REQUISIÇÕES APERIÓDICAS ///////////////////////
typedef void (*job_func_t)(void *arg);

typedef struct job
{
    job_func_t func;
    void *arg;
    struct job *next;
} job_t;

static job_t *queue_head = NULL;
static job_t *queue_tail = NULL;
static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;

// Criacao das filas de pedidos
// Funcao que enfilera as jobs
void enqueue_job(job_func_t f, void *arg)
{
    job_t *j = malloc(sizeof(job_t));
    j->func = f;
    j->arg = arg;
    j->next = NULL;

    pthread_mutex_lock(&queue_mutex);
    if (queue_tail)
        queue_tail->next = j;
    else
        queue_head = j;
    queue_tail = j;
    pthread_mutex_unlock(&queue_mutex);
}

// Funcao que desenfilera as jobs
job_t *dequeue_job(void)
{
    job_t *j = queue_head;
    if (!j)
        return NULL;
    queue_head = j->next;
    if (!queue_head)
        queue_tail = NULL;
    return j;
}

/////////////////////// UTILITÁRIOS DE TEMPO ///////////////////////
static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= NS_PER_SEC)
    {
        t->tv_nsec -= NS_PER_SEC;
        t->tv_sec += 1;
    }
}

static long timespec_diff_ns(const struct timespec *end,
                             const struct timespec *start)
{
    return (end->tv_sec - start->tv_sec) * NS_PER_SEC + (end->tv_nsec - start->tv_nsec);
}

// Tipo de carga simulada: 0 = nanosleep (dorme), 1 = laco de NOP (ocupa a CPU)
// Escolhido na linha de comando: ./servidor_aperiodico nop
static int g_modo_nop = 0;
static void busy_us(long us);

/////////////////////// Pequeno "trabalho" de 5ms para simular carga
static void busy_5ms(void)
{
    busy_us(5000);
}

/////////////////////// JOB APERIÓDICO: NAV_PLAN ///////////////////////
// Atitude simulada (usada na telemetria)
static float g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;

static int64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000LL + t.tv_nsec / 1000;
}

// Simula carga por "us" microssegundos.
// Modo padrao: nanosleep, como o busy_5ms do professor (a thread dorme).
// Modo nop: laco de NOP, a thread ocupa a CPU durante todo o tempo.
static void busy_us(long us)
{
    if (g_modo_nop)
    {
        int64_t inicio = now_us();
        while ((now_us() - inicio) < us)
            __asm__ __volatile__("nop");
    }
    else
    {
        struct timespec req = {0};
        req.tv_nsec = us * 1000L;
        nanosleep(&req, NULL);
    }
}

// Tipo de evento tratado pelo NAV_PLAN
typedef enum
{
    EV_NAV = 0,
    EV_TEL = 1
} evt_t;

// Argumento do job: tipo do evento e instante em que o pedido chegou
typedef struct
{
    evt_t ev;
    int64_t chegada_us;
} nav_req_t;

// Estatisticas globais (em ms), atualizadas dentro do nav_plan_job.
// Indice 0 = NAV, 1 = TEL
static long job_n[2] = {0, 0};
static double job_soma_exec[2], job_soma_espera[2], job_soma_resp[2];
static double job_min_exec[2], job_min_espera[2], job_min_resp[2];
static double job_max_exec[2], job_max_espera[2], job_max_resp[2];
static pthread_mutex_t stats_mutex = PTHREAD_MUTEX_INITIALIZER;

// arg = nav_req_t alocado pelo gerador (liberado no fim do job)
void nav_plan_job(void *arg)
{
    nav_req_t *req = (nav_req_t *)arg;
    evt_t ev = req->ev;
    int64_t chegada_us = req->chegada_us;
    int64_t task_start_us = now_us();

    if (ev == EV_NAV)
    {
        printf("NAV_PLAN: Evento NAV recebido\n");
        busy_us(3500); // cálculo de rota ~3.5 ms
    }
    else if (ev == EV_TEL)
    {
        printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f\n", g_roll, g_pitch, g_yaw);
        busy_us(500); // processamento ~0.5 ms
    }

    int64_t task_end_us = now_us();
    printf("NAV_PLAN: chegada=%.2fms inicio=%.2fms fim=%.2fms exec=%.2fms espera=%.2fms resposta=%.2fms\n",
           chegada_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
           (task_end_us - task_start_us) / 1000.0,
           (task_start_us - chegada_us) / 1000.0,
           (task_end_us - chegada_us) / 1000.0);

    // atualiza as estatisticas globais deste tipo de evento
    double exec_ms = (task_end_us - task_start_us) / 1000.0;
    double espera_ms = (task_start_us - chegada_us) / 1000.0;
    double resp_ms = (task_end_us - chegada_us) / 1000.0;

    pthread_mutex_lock(&stats_mutex);
    if (job_n[ev] == 0)
    {
        job_min_exec[ev] = job_max_exec[ev] = exec_ms;
        job_min_espera[ev] = job_max_espera[ev] = espera_ms;
        job_min_resp[ev] = job_max_resp[ev] = resp_ms;
    }
    if (exec_ms < job_min_exec[ev]) job_min_exec[ev] = exec_ms;
    if (exec_ms > job_max_exec[ev]) job_max_exec[ev] = exec_ms;
    if (espera_ms < job_min_espera[ev]) job_min_espera[ev] = espera_ms;
    if (espera_ms > job_max_espera[ev]) job_max_espera[ev] = espera_ms;
    if (resp_ms < job_min_resp[ev]) job_min_resp[ev] = resp_ms;
    if (resp_ms > job_max_resp[ev]) job_max_resp[ev] = resp_ms;
    job_soma_exec[ev] += exec_ms;
    job_soma_espera[ev] += espera_ms;
    job_soma_resp[ev] += resp_ms;
    job_n[ev]++;
    pthread_mutex_unlock(&stats_mutex);

    free(arg);
}

/////////////////////// SERVIDOR PERIÓDICO ///////////////////////
typedef struct
{
    long period_ns; // Ts
    long budget_ns; // Cs
} server_params_t;

void *server_thread(void *arg)
{
    server_params_t *params = (server_params_t *)arg;
    long Ts = params->period_ns;
    long Cs = params->budget_ns;

    struct timespec next_release;
    clock_gettime(CLOCK_MONOTONIC, &next_release);

    while (1)
    {
        timespec_add_ns(&next_release, Ts); // próxima liberação
        struct timespec start_period;
        clock_gettime(CLOCK_MONOTONIC, &start_period);

        long elapsed = 0;

        while (elapsed < Cs)
        {
            pthread_mutex_lock(&queue_mutex);
            job_t *j = dequeue_job();
            pthread_mutex_unlock(&queue_mutex);

            if (!j)
            {
                // nada para fazer neste período
                break;
            }

            struct timespec t_before, t_after;
            clock_gettime(CLOCK_MONOTONIC, &t_before);
            j->func(j->arg); // executa job
            clock_gettime(CLOCK_MONOTONIC, &t_after);

            elapsed = timespec_diff_ns(&t_after, &start_period);
            free(j);
        }

        // Dorme até o próximo período (tempo absoluto)
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_release, NULL);
    }
    return NULL;
}

/////////////////////// TAREFAS PERIÓDICAS (T1..T4) ///////////////////////
typedef struct
{
    const char *name;
    long period_ns;
} periodic_task_params_t;

void *periodic_task(void *arg)
{
    periodic_task_params_t *p = (periodic_task_params_t *)arg;
    struct timespec next_release;
    clock_gettime(CLOCK_MONOTONIC, &next_release);

    while (1)
    {
        timespec_add_ns(&next_release, p->period_ns);
        printf("[%s] executando\n", p->name);
        busy_5ms(); // cada tarefa periódica consome ~5ms
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_release, NULL);
    }
    return NULL;
}

/////////////////////// GERADOR DE REQUISIÇÕES APERIÓDICAS ///////////////////////
void *aperiodic_generator(void *arg)
{
    (void)arg;
    int counter = 0;
    while (1)
    {
        // gera um job a cada 30–120 ms
        int delay_ms = 30 + rand() % 90;
        struct timespec req = {0};
        req.tv_nsec = delay_ms * NS_PER_MS;
        nanosleep(&req, NULL);

        nav_req_t *p = malloc(sizeof(nav_req_t));
        p->ev = (counter++ % 2 == 0) ? EV_NAV : EV_TEL;
        p->chegada_us = now_us(); // instante de chegada do pedido
        enqueue_job(nav_plan_job, p);
    }
    return NULL;
}

/////////////////////// MAIN ///////////////////////
static void create_rt_thread(pthread_t *th, void *(*func)(void *), void *arg, int priority)
{
    pthread_attr_t attr;
    struct sched_param sp;

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &sp);

    int ret = pthread_create(th, &attr, func, arg);
    if (ret != 0)
    {
        fprintf(stderr, "pthread_create: %s\n", strerror(ret));
    }
    pthread_attr_destroy(&attr);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "nop") == 0)
        g_modo_nop = 1;
    printf("Modo de carga: %s\n", g_modo_nop ? "NOP (ocupa a CPU)" : "nanosleep (dorme)");

    srand(time(NULL));

    // ----------------- Servidor periódico -----------------
    server_params_t server_params = {
        .period_ns = 50 * NS_PER_MS, // Ts = 50ms
        .budget_ns = 20 * NS_PER_MS  // Cs = 20ms por período
    };
    pthread_t th_server;
    create_rt_thread(&th_server, server_thread, &server_params, 60);

    // ----------------- Tarefas periódicas -----------------
    periodic_task_params_t tparams[4] = {
        {"T1", 40 * NS_PER_MS},
        {"T2", 80 * NS_PER_MS},
        {"T3", 120 * NS_PER_MS},
        {"T4", 160 * NS_PER_MS}};

    pthread_t th_tasks[4];
    for (int i = 0; i < 4; ++i)
    {
        create_rt_thread(&th_tasks[i], periodic_task, &tparams[i], 50);
    }

    // ----------------- Gerador aperiódico -----------------
    pthread_t th_gen;
    // gerador pode ter prioridade menor (não precisa ser RT)
    pthread_create(&th_gen, NULL, aperiodic_generator, NULL);

    printf("Rodando por 10 segundos...\n");
    sleep(10);

    // ----------------- Resumo (so imprime o que o job acumulou) -----------------
    const char *nomes[2] = {"NAV", "TEL"};
    pthread_mutex_lock(&stats_mutex);
    printf("\n===== RESUMO (ms) =====\n");
    for (int e = 0; e < 2; e++)
    {
        long n = job_n[e] ? job_n[e] : 1;
        printf("%s: %ld requisicoes\n", nomes[e], job_n[e]);
        printf("  exec     min=%.2f  media=%.2f  max=%.2f\n", job_min_exec[e], job_soma_exec[e] / n, job_max_exec[e]);
        printf("  espera   min=%.2f  media=%.2f  max=%.2f\n", job_min_espera[e], job_soma_espera[e] / n, job_max_espera[e]);
        printf("  resposta min=%.2f  media=%.2f  max=%.2f\n", job_min_resp[e], job_soma_resp[e] / n, job_max_resp[e]);
    }
    pthread_mutex_unlock(&stats_mutex);

    printf("Encerrando (para exemplo simples vamos apenas sair do processo).\n");
    // Em código real você sinalizaria as threads para sair, etc.
    return 0;
}
