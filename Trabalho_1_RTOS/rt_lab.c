#define _GNU_SOURCE
#include <pthread.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <stdatomic.h>

/* =========================
   Utilitários de tempo
   ========================= */
static inline long ns_from_ms(long ms){ return ms*1000000L; }
static inline double ms_from_ns(long long ns){ return (double)ns/1e6; }

static inline struct timespec ts_add(struct timespec a, long ns){
    struct timespec r = a;
    r.tv_nsec += ns;
    while(r.tv_nsec >= 1000000000L){ r.tv_nsec -= 1000000000L; r.tv_sec++; }
    return r;
}
static inline long long ts_diff_ns(struct timespec end, struct timespec start){
    long long s = (long long)(end.tv_sec - start.tv_sec)*1000000000LL;
    s += (end.tv_nsec - start.tv_nsec);
    return s;
}
static inline struct timespec now_monotonic(void){
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t;
}

/* =========================
   Configuração das tarefas
   ========================= */
typedef enum { PERIODIC, SPORADIC, APERIODIC } recurrence_t;
typedef enum { DEADLINE_HARD, DEADLINE_FIRM, DEADLINE_SOFT } dclass_t;

typedef struct {
    const char* name;
    recurrence_t kind;
    dclass_t     dclass;
    long         period_ms;    // Para periódicas (P)
    long         min_ia_ms;    // Intervalo mínimo (esp./ap.)
    long         wcet_ms;      // C (simulado)
    long         deadline_ms;  // D relativo
    int          prio;         // Maior = mais prioritária
    // métricas
    atomic_long jobs;
    atomic_long misses;
    atomic_long sum_resp_ns;
} task_cfg_t;

typedef struct {
    task_cfg_t* cfg;
    pthread_t   thr;
    struct timespec start_time;
    struct timespec next_release;
    atomic_int stop;
} task_t;

/* Tarefas:
   - T1: periódica, hard, P=50ms, C~10ms, D=50ms, prioridade alta
   - T2: periódica, soft,  P=80ms, C~20ms, D=80ms, prioridade média
   - T3: esporádica, firm, min IA=200ms, C~15ms, D=60ms, prioridade baixa
*/
static task_cfg_t g_cfg[] = {
    { "T1", PERIODIC,  DEADLINE_HARD, 50,  0, 10, 50, 60, 0, 0, 0 },
    { "T2", PERIODIC,  DEADLINE_SOFT, 80,  0, 20, 80, 80, 0, 0, 0 },
    { "T3", SPORADIC,  DEADLINE_FIRM,  0, 100, 15, 60, 40, 0, 0, 0 },
};
enum { NT = sizeof(g_cfg)/sizeof(g_cfg[0]) };
static task_t g_tasks[NT];

/* Seção crítica compartilhada (exclusão mútua) */
static pthread_mutex_t g_shared_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_shared_value = 0;

/* =========================
   Simulação do WCET
   =========================
   A "execução" consiste em:
   - Pequena seção crítica (mutex) para gerar bloqueio/interferência
   - Busy-wait / sleep preciso para consumir C ms
*/
static void burn_cpu_ms(long ms){
    /* Usamos clock_nanosleep para precisão e estabilidade,
       mas adicionamos um micro busy-spin curto para simular
       comportamento menos previsível. */
    struct timespec t = { .tv_sec = ms/1000, .tv_nsec = (ms%1000)*1000000L };
    /* micro jitter */
    for (volatile int i=0; i<10000; ++i);
    clock_nanosleep(CLOCK_MONOTONIC, 0, &t, NULL);
}

/* Retorna 1 se perdeu deadline; 0 caso contrário. Atualiza métricas. */
static int run_job(task_cfg_t* cfg, const struct timespec arrival)
{
    struct timespec job_start = now_monotonic();

    /* Seção crítica curta (exclusão mútua) — simula recurso compartilhado */
    pthread_mutex_lock(&g_shared_mutex);
    int local = g_shared_value;
    local += 1;                 // trabalho "no recurso"
    g_shared_value = local;
    /* pequena permanência na SC para evidenciar bloqueio */
    struct timespec sc_sleep = { .tv_sec = 0, .tv_nsec = ns_from_ms(2) };
    clock_nanosleep(CLOCK_MONOTONIC, 0, &sc_sleep, NULL);
    pthread_mutex_unlock(&g_shared_mutex);

    /* Corpo principal: consome C ms (WCET aproximado) */
    burn_cpu_ms(cfg->wcet_ms);

    struct timespec finish = now_monotonic();

    long long resp_ns = ts_diff_ns(finish, arrival);
    atomic_fetch_add(&cfg->sum_resp_ns, resp_ns);
    atomic_fetch_add(&cfg->jobs, 1);

    /* Verifica deadline relativo */
    long long d_ns = ns_from_ms(cfg->deadline_ms);
    long long fin_vs_deadline = ts_diff_ns(finish, arrival);
    int missed = fin_vs_deadline > d_ns;

    if (missed) atomic_fetch_add(&cfg->misses, 1);
    
    /* Tratamento especial para deadlines Firm */
    if (cfg->dclass == DEADLINE_FIRM && missed) {
    atomic_fetch_add(&cfg->misses, 1);
    return 1; // descarta execução atrasada
    }
    return missed;
}


/* =========================
   Disparo por tempo (time-triggered) – T1 e T2
   ========================= */
static void* periodic_worker(void* arg){
    task_t* t = (task_t*)arg;
    task_cfg_t* cfg = t->cfg;

    /* Define política/ prioridade (best-effort: pode falhar sem root/cap) */
    struct sched_param sp = { .sched_priority = cfg->prio };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        // Fallback silencioso: SCHED_OTHER
    }

    t->start_time = now_monotonic();
    t->next_release = t->start_time;

    while (!atomic_load(&t->stop)) {
        /* Proxima liberação */
        t->next_release = ts_add(t->next_release, ns_from_ms(cfg->period_ms));

        /* Chegada = instante de liberação */
        struct timespec arrival = t->next_release;

        /* Dorme até a liberação (tick) */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t->next_release, NULL);

        /* Executa o job */
        run_job(cfg, arrival);
    }
    return NULL;
}

/* =========================
   Disparo por evento (event-triggered) – T3
   =========================
   Gera chegadas esporádicas com intervalo mínimo.
*/
static void* sporadic_worker(void* arg){
    task_t* t = (task_t*)arg;
    task_cfg_t* cfg = t->cfg;

    struct sched_param sp = { .sched_priority = cfg->prio };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        // fallback SCHED_OTHER
    }

    struct timespec last = now_monotonic();
    while (!atomic_load(&t->stop)) {
        /* Gera um atraso >= min_ia_ms com alguma variação */
        long jitter = (rand()%80) - 40; // +/- 40ms
        long wait_ms = cfg->min_ia_ms + (jitter>=0? jitter: -jitter/2); // evita ficar < min
        if (wait_ms < (long)cfg->min_ia_ms) wait_ms = cfg->min_ia_ms;

        struct timespec wake = ts_add(last, ns_from_ms(wait_ms));
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, NULL);
        last = wake;

        /* Chegada instantânea (evento) */
        struct timespec arrival = now_monotonic();
        run_job(cfg, arrival);
    }
    return NULL;
}

/* =========================
   Sumário de métricas
   ========================= */
static void print_summary(double seconds){
    puts("\n======= SUMÁRIO =======");
    printf("Duração do experimento: %.1f s\n", seconds);
    printf("Valor final do recurso compartilhado: %d\n\n", g_shared_value);

    for (int i=0;i<NT;i++){
        task_cfg_t* c = &g_cfg[i];
        long jobs = atomic_load(&c->jobs);
        long misses = atomic_load(&c->misses);
        long long sumresp = atomic_load(&c->sum_resp_ns);
        double avg_ms = jobs>0 ? ms_from_ns(sumresp / (jobs)) : 0.0;

        const char* k = (c->kind==PERIODIC)?"periodic":(c->kind==SPORADIC)?"sporadic":"aperiodic";
        const char* dc= (c->dclass==DEADLINE_HARD)?"hard":(c->dclass==DEADLINE_FIRM)?"firm":"soft";

        printf("[%s] kind=%s, deadline=%s, C≈%ldms, D=%ldms",
               c->name, k, dc, c->wcet_ms, c->deadline_ms);
        if (c->kind==PERIODIC) printf(", P=%ldms", c->period_ms);
        if (c->kind==SPORADIC) printf(", minIA=%ldms", c->min_ia_ms);
        printf(", prio=%d\n", c->prio);

        printf("   jobs=%ld, misses=%ld (%.1f%%), resp_médio=%.2f ms\n",
               jobs, misses, jobs? (100.0*misses/jobs):0.0, avg_ms);
    }
    puts("=======================\n");
}

/* =========================
   Main
   ========================= */
int main(int argc, char** argv){
    int duration_sec = (argc>1)? atoi(argv[1]) : 10;
    if (duration_sec <= 0) duration_sec = 10;
    srand((unsigned)time(NULL));

    // Cria workers
    for (int i=0;i<NT;i++){
        g_tasks[i].cfg = &g_cfg[i];
        g_tasks[i].start_time = now_monotonic();
        atomic_store(&g_tasks[i].stop, 0);
        if (g_cfg[i].kind==PERIODIC)
            pthread_create(&g_tasks[i].thr, NULL, periodic_worker, &g_tasks[i]);
        else
            pthread_create(&g_tasks[i].thr, NULL, sporadic_worker, &g_tasks[i]);
    }

    // Roda pelo tempo definido
    sleep(duration_sec);

    // Para e junta as threads
    for (int i=0;i<NT;i++) atomic_store(&g_tasks[i].stop, 1);
    for (int i=0;i<NT;i++) pthread_join(g_tasks[i].thr, NULL);

    print_summary((double)duration_sec);
    return 0;
}