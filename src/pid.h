#pragma once

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float previous_error;

    float dt;
} PID;

void PID_init(PID *pid, float kp, float ki, float kd, float dt);
float PID_update(PID *pid, float target, float measured);