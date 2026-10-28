#pragma once
#include <stdint.h>
typedef struct { uint64_t off_cpu_us; uint32_t switches; } experiment_trace_sample_t;
void experiment_trace_begin(unsigned index);
void experiment_trace_end(unsigned index);
experiment_trace_sample_t experiment_trace_read(const char *name);

void experiment_trace_reset(void);

void experiment_trace_pause(void);
