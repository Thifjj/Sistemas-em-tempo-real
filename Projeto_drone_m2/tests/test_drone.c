#include <assert.h>
#include <stdio.h>
#include "drone_sim.h"
#include "timing_metrics.h"
int main(void)
{
    drone_t d; drone_init(&d);
    /* Bias de gyro nao pode causar deriva sem limite no filtro complementar. */
    for(int i=0;i<10000;i++) { drone_fuse(&d,.005f); drone_control(&d,d.estimate,.005f); }
    assert(fabsf(d.estimate.roll)<1 && fabsf(d.estimate.pitch)<1);
    assert(fabsf(d.estimate.altitude-1)<.1f);
    drone_navigate(&d); assert(d.waypoint==1 && d.target.altitude==1.5f);
    for(int i=0;i<20000;i++) { drone_fuse(&d,.005f); drone_control(&d,d.estimate,.005f); }
    assert(fabsf(d.estimate.altitude-1.5f)<.15f);
    for(int i=0;i<3;i++) drone_navigate(&d);
    assert(d.waypoint==0);
    drone_fail_safe(&d);
    for(int i=0;i<100;i++) { drone_navigate(&d); drone_control(&d,d.estimate,.005f); }
    assert(d.emergency && d.motors.m1==0 && d.motors.m2==0 && d.motors.m3==0 && d.motors.m4==0);
    drone_pid_t pid={0};
    for(int i=0;i<10000;i++) assert(fabsf(pid_step(&pid,100,.005f,1,1,1,10))<=10);
    assert(fabsf(pid.integral)<.01f);
    timing_metrics_t m={0};
    for(int i=0;i<9;i++) timing_add(&m,0,10,100,100,true);
    assert(m.windows==0);
    timing_add(&m,0,20,101,100,true);
    assert(m.windows==1 && m.violations==0 && m.misses==1);
    timing_add(&m,0,30,102,100,true);
    assert(m.windows==2 && m.violations==1 && m.response_max==102 && m.latency_max==30);
    assert(timing_hwm99_bin(&m)==2);
    assert(m.worst_event_us==0 && m.worst_start_us==30 && m.worst_end_us==102);
    assert(m.last_start_us==30 && m.last_end_us==102);
    timing_add(&m,0,0,100001,100,false);
    assert(m.histogram[HIST_BINS-1]==1);
    puts("OK: filtro, PID, altitude, rota, fail-safe, metricas e janelas deslizantes (9,10)-firm");
}
