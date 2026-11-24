#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/////////////////////// Fila de requisições aperiódicas ///////////////////////////
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
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;

/////////////////////// Enfileira um job (chamado pelas tarefas aperiódicas) ///////////////////////
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
    pthread_cond_signal(&queue_cond);
    pthread_mutex_unlock(&queue_mutex);
}

/////////////////////// Retira um job da fila (usado pelo servidor) ///////////////////////
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

/////////////////////// soma nanossegundos a um timespec (periodo) ///////////////////////
static void timespec_add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L)
    {
        t->tv_nsec -= 1000000000L;
        t->tv_sec += 1;
    }
}

typedef struct
{
    long period_ns; // Ts (por ex. 10ms = 10*10^6 ns)
    long budget_ns; // Cs (pode ser <= Ts*Umax)
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
        /////////////////////// define momento da próxima ativação
        timespec_add_ns(&next_release, Ts);

        /////////////////////// início do período
        struct timespec start_period;
        clock_gettime(CLOCK_MONOTONIC, &start_period);

        long consumed_ns = 0;

        while (consumed_ns < Cs)
        {
            /////////////////////// pega um job, se existir
            pthread_mutex_lock(&queue_mutex);
            while (!queue_head)
            {
                /////////////////////// se não há jobs, pode sair do loop de serviço
                pthread_mutex_unlock(&queue_mutex);
                goto end_of_service;
            }
            job_t *j = dequeue_job();
            pthread_mutex_unlock(&queue_mutex);

            /////////////////////// mede tempo antes/depois de executar o job
            struct timespec t_before, t_after;
            clock_gettime(CLOCK_MONOTONIC, &t_before);
            j->func(j->arg); // executa requisição aperiódica
            clock_gettime(CLOCK_MONOTONIC, &t_after);

            /////////////////////// atualiza orçamento consumido
            long dt = (t_after.tv_sec - t_before.tv_sec) * 1000000000L +
                      (t_after.tv_nsec - t_before.tv_nsec);
            consumed_ns += dt;

            free(j);

            if (consumed_ns >= Cs)
            {
                // orçamento esgotado neste período
                break;
            }
        }

    end_of_service:
        /////////////////////// Dorme até o instante absoluto da próxima liberação
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                        &next_release, NULL);
    }
    return NULL;
}

void start_server_thread()
{
    pthread_t th;
    pthread_attr_t attr;
    struct sched_param sp;

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO); // ou SCHED_RR

    sp.sched_priority = 60; // prioridade RT (ajuste conforme sistema)
    pthread_attr_setschedparam(&attr, &sp);

    static server_params_t params = {
        .period_ns = 10L * 1000000L, // Ts = 10 ms
        .budget_ns = 3L * 1000000L   // Cs = 3 ms por período
    };

    int r = pthread_create(&th, &attr, server_thread, &params);
    if (r != 0)
    {
        // Sem sudo o SCHED_FIFO é negado (EPERM) e o servidor não é criado
        fprintf(stderr, "Erro ao criar servidor (%d). Execute com sudo!\n", r);
        exit(1);
    }
    pthread_attr_destroy(&attr);
}

// ======================================================
// TASK NAV_PLAN (adaptada do M2 para PC / pthread)
// ======================================================

// Eventos possíveis da NAV_PLAN
typedef enum
{
    EV_NAV = 1,
    EV_TEL = 2
} evt_t;

// Estado simulado (usado na telemetria)
static float g_roll = 0, g_pitch = 0, g_yaw = 0;

// Tempo monotônico em microssegundos (substitui esp_timer_get_time)
static int64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000LL + t.tv_nsec / 1000;
}

// Ocupa a CPU pelo tempo indicado (igual ao do M2)
static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = now_us();
    while ((now_us() - start) < us)
    {
        __asm__ __volatile__("nop");
    }
}

// Argumento do job: tipo do evento e instante em que o pedido chegou
typedef struct
{
    evt_t ev;
    int64_t chegada_us;
} nav_req_t;

// Job da NAV_PLAN: executado pelo servidor (arg = nav_req_t alocado)
void job_nav_plan(void *arg)
{
    nav_req_t *req = (nav_req_t *)arg;
    evt_t ev = req->ev;
    int64_t task_start_us = now_us();

    // --------------------------
    // Evento de navegação
    // --------------------------
    if (ev == EV_NAV)
    {
        // Simula cálculo de rota
        // aproximadamente 3.5 ms
        printf("NAV_PLAN: Evento NAV recebido\n");
        cpu_tight_loop_us(3500);
    }

    // --------------------------
    // Evento de telemetria
    // --------------------------
    else if (ev == EV_TEL)
    {
        printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f\n", g_roll, g_pitch, g_yaw);

        // Simula processamento
        cpu_tight_loop_us(500);
    }

    int64_t task_end_us = now_us();
    printf("NAV_PLAN: chegada=%.2fms inicio=%.2fms fim=%.2fms exec=%.2fms espera=%.2fms resposta=%.2fms\n",
           req->chegada_us / 1000.0, task_start_us / 1000.0, task_end_us / 1000.0,
           (task_end_us - task_start_us) / 1000.0,
           (task_start_us - req->chegada_us) / 1000.0,
           (task_end_us - req->chegada_us) / 1000.0);

    free(arg);
}

void requisicao_aperiodica(evt_t ev)
{
    nav_req_t *p = malloc(sizeof(nav_req_t));
    p->ev = ev;
    p->chegada_us = now_us(); // instante em que o pedido aperiódico chegou
    enqueue_job(job_nav_plan, p);
}

// ======================================================
// MAIN: liga o servidor e dispara pedidos aperiódicos
// (substitui o touch / rede do M2)
// ======================================================
int main(void)
{
    start_server_thread();

    // 20 pedidos alternando NAV e TEL, em intervalos irregulares (5 a 30 ms)
    for (int i = 0; i < 20; i++)
    {
        requisicao_aperiodica((i % 2 == 0) ? EV_NAV : EV_TEL);
        usleep((5 + (i * 7) % 26) * 1000);
    }

    sleep(1); // dá tempo do servidor esvaziar a fila
    return 0;
}
