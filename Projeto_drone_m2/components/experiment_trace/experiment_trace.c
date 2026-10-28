#include "experiment_trace.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>
#include "esp_trace_registry.h"

/* Samples stay in RAM and leave through the application log queue. */
static const esp_trace_encoder_vtable_t experiment_encoder = {0};
ESP_TRACE_REGISTER_ENCODER("ext", &experiment_encoder);
static struct { void *task; int active; int64_t out; experiment_trace_sample_t sample; } slots[6];
static const char *names[] = {"FUS_IMU", "CTRL_ATT", "NAV_PLAN", "FS_TASK", "MONITOR_TASK", "TIME_TASK"};
static bool enabled=true;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
void experiment_trace_begin(unsigned i) {
    void *task = xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&lock);
    slots[i].task=task; slots[i].active=enabled; slots[i].out=0;
    portEXIT_CRITICAL(&lock);
}
void experiment_trace_end(unsigned i) {
    portENTER_CRITICAL(&lock); slots[i].active=0; portEXIT_CRITICAL(&lock);
}
/* No queue, allocation or task API inside scheduler hooks. */
void experiment_trace_switch(void *task, int entering) {
    int64_t now=esp_timer_get_time();
    portENTER_CRITICAL(&lock);
    for(unsigned i=0;i<6;i++) if(slots[i].task==task && slots[i].active) {
        if(!entering) { slots[i].out=now; slots[i].sample.switches++; }
        else if(slots[i].out) { slots[i].sample.off_cpu_us+=now-slots[i].out; slots[i].out=0; }
        break;
    }
    portEXIT_CRITICAL(&lock);
}
experiment_trace_sample_t experiment_trace_read(const char *name) {
    experiment_trace_sample_t result={0};
    for(unsigned i=0;i<6;i++) if(strcmp(name,names[i])==0) {
        portENTER_CRITICAL(&lock); result=slots[i].sample; portEXIT_CRITICAL(&lock); break;
    }
    return result;
}

void experiment_trace_reset(void) {
    portENTER_CRITICAL(&lock);
    enabled=true;
    for(unsigned i=0;i<6;i++) { memset(&slots[i].sample,0,sizeof(slots[i].sample)); slots[i].out=0; }
    portEXIT_CRITICAL(&lock);
}

void experiment_trace_pause(void) {
    portENTER_CRITICAL(&lock);
    enabled=false;
    for(unsigned i=0;i<6;i++) { slots[i].active=0; slots[i].out=0; }
    portEXIT_CRITICAL(&lock);
}
