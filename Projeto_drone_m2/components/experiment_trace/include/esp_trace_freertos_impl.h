#pragma once
void experiment_trace_switch(void *task, int entering);
#define traceTASK_SWITCHED_IN() experiment_trace_switch((void *)pxCurrentTCBs[xPortGetCoreID()], 1)
#define traceTASK_SWITCHED_OUT() experiment_trace_switch((void *)pxCurrentTCBs[xPortGetCoreID()], 0)
