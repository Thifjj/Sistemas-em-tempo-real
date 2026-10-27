#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#define MK_M 9
#define MK_K 10
/* Histograma: HWM99 aproximado, resolucao de 100 us ate 100 ms. */
#define HIST_BINS 1002
#define HIST_US 100

typedef struct {
    uint32_t runs, misses, drops, windows, violations;
    uint64_t response_max, execution_max, latency_max, jitter_max, response_sum;
    uint32_t histogram[HIST_BINS];
    uint16_t history;
    unsigned filled;
} timing_metrics_t;
static inline void timing_add(timing_metrics_t *s, int64_t event, int64_t start,
                              int64_t end, uint32_t deadline, bool soft)
{
    uint64_t r = end > event ? end-event : 0;
    uint64_t c = end > start ? end-start : 0;
    uint64_t l = start > event ? start-event : 0;
    s->runs++; s->misses += r > deadline; s->response_sum += r;
    if(r>s->response_max) s->response_max=r;
    if(c>s->execution_max) s->execution_max=c;
    if(l>s->latency_max) s->latency_max=l;
    /* Jitter de liberacao: desvio absoluto do inicio em relacao ao release. */
    uint64_t jitter = start >= event ? start-event : event-start;
    if(jitter>s->jitter_max) s->jitter_max=jitter;
    unsigned bin=(unsigned)((r+HIST_US-1)/HIST_US);
    s->histogram[bin < HIST_BINS ? bin : HIST_BINS-1]++;
    if(soft) {
        s->history=((s->history<<1)|(r<=deadline)) & ((1U<<MK_K)-1);
        if(s->filled<MK_K) s->filled++;
        if(s->filled==MK_K) {
            unsigned successes=0;
            for(unsigned i=0;i<MK_K;i++) successes+=(s->history>>i)&1;
            s->windows++; s->violations += successes<MK_M;
        }
    }
}
static inline unsigned timing_hwm99_bin(const timing_metrics_t *s)
{
    if (!s->runs) return 0;
    uint64_t target=((uint64_t)s->runs*99+99)/100, accumulated=0;
    for(unsigned i=0;i<HIST_BINS;i++) if((accumulated+=s->histogram[i])>=target) return i;
    return HIST_BINS-1;
}
