#pragma once
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* Modelo didatico: unidades em graus, metros, segundos e throttle em %. */
typedef struct { float roll, pitch, yaw, altitude, x, y; } state_t;
typedef struct { float m1, m2, m3, m4; } esc_output_t;
typedef struct { float integral, previous; bool initialized; } drone_pid_t;
typedef struct {
    state_t truth, estimate, target;
    esc_output_t motors;
    drone_pid_t pid[4];
    unsigned waypoint, samples;
    bool emergency;
} drone_t;
static inline float limit(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
static inline float pid_step(drone_pid_t *p, float error, float dt, float kp, float ki, float kd, float bound)
{
    float integral = limit(p->integral + error * dt, -20, 20);
    float derivative = p->initialized ? (error - p->previous) / dt : 0;
    float raw = kp * error + ki * integral + kd * derivative;
    /* Anti-windup: integra apenas quando nao aumenta a saturacao. */
    if (fabsf(raw) <= bound || raw * error < 0) p->integral = integral;
    p->previous = error; p->initialized = true;
    return limit(raw, -bound, bound);
}
static inline void drone_init(drone_t *d)
{
    memset(d, 0, sizeof(*d));
    d->truth.altitude = d->estimate.altitude = d->target.altitude = 1;
    d->motors = (esc_output_t){50, 50, 50, 50};
}
static inline void drone_fuse(drone_t *d, float dt)
{
    esc_output_t m = d->motors;
    float old_roll = d->truth.roll, old_pitch = d->truth.pitch, old_yaw = d->truth.yaw;
    d->truth.roll += ((m.m1-m.m2-m.m3+m.m4)*0.5f - d->truth.roll) * dt;
    d->truth.pitch += ((m.m1+m.m2-m.m3-m.m4)*0.5f - d->truth.pitch) * dt;
    d->truth.yaw += ((-m.m1+m.m2-m.m3+m.m4)*0.5f - d->truth.yaw*0.2f) * dt;
    d->truth.altitude = fmaxf(0, d->truth.altitude + ((m.m1+m.m2+m.m3+m.m4)*0.25f-50)*0.02f*dt);
    d->truth.x += d->truth.pitch*0.1f*dt;
    d->truth.y += d->truth.roll*0.1f*dt;
    /* Giroscopio com bias e acelerometro/magnetometro com ruido deterministico.
       O filtro complementar corrige a deriva da integracao giroscopica. */
    float noise = 0.2f * sinf(++d->samples * 0.1f);
    float alpha = 0.25f / (0.25f + dt);
    d->estimate.roll = alpha*(d->estimate.roll + (d->truth.roll-old_roll) + 0.02f*dt)
                      + (1-alpha)*(d->truth.roll+noise);
    d->estimate.pitch = alpha*(d->estimate.pitch + (d->truth.pitch-old_pitch) - 0.02f*dt)
                       + (1-alpha)*(d->truth.pitch-noise);
    d->estimate.yaw = alpha*(d->estimate.yaw + (d->truth.yaw-old_yaw) + 0.01f*dt)
                     + (1-alpha)*(d->truth.yaw+noise*0.5f);
    d->estimate.altitude += (1-alpha)*(d->truth.altitude-d->estimate.altitude);
    d->estimate.x = d->truth.x; d->estimate.y = d->truth.y;
}
static inline void drone_control(drone_t *d, state_t sample, float dt)
{
    /* Emergencia fica travada ate reiniciar a placa; nenhuma tarefa rearma motores. */
    if (d->emergency) { d->motors = (esc_output_t){0}; return; }
    float r = pid_step(&d->pid[0], d->target.roll-sample.roll, dt, .8f,.1f,.03f,15);
    float p = pid_step(&d->pid[1], d->target.pitch-sample.pitch, dt, .8f,.1f,.03f,15);
    float y = pid_step(&d->pid[2], d->target.yaw-sample.yaw, dt, .4f,.05f,.01f,10);
    float t = 50 + pid_step(&d->pid[3], d->target.altitude-sample.altitude, dt, 15,2,1,20);
    d->motors = (esc_output_t){limit(t+p+r-y,0,100),limit(t+p-r+y,0,100),
                              limit(t-p-r-y,0,100),limit(t-p+r+y,0,100)};
}
static inline void drone_navigate(drone_t *d)
{
    static const float route[][3] = {{1,0,1.5f},{1,1,2},{0,1,1.5f},{0,0,1}};
    if (d->emergency) return;
    const float *w = route[d->waypoint];
    d->target.pitch = limit((w[0]-d->estimate.x)*3,-5,5);
    d->target.roll = limit((w[1]-d->estimate.y)*3,-5,5);
    d->target.altitude = w[2];
    d->waypoint = (d->waypoint+1)%4;
}
static inline void drone_fail_safe(drone_t *d)
{
    d->emergency = true;
    d->motors = (esc_output_t){0};
    memset(d->pid,0,sizeof(d->pid));
}
