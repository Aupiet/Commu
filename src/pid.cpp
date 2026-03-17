#include "pid.h"

void PID_init(PID *pid, float kp, float ki, float kd, float dt)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;

    pid->integral = 0;
    pid->previous_error = 0;
}

float PID_update(PID *pid, float target, float measured)
{
    float error = target - measured;

    pid->integral += error * pid->dt;

    float derivative = (error - pid->previous_error) / pid->dt;

    float output =
        pid->Kp * error +
        pid->Ki * pid->integral +
        pid->Kd * derivative;

    pid->previous_error = error;

    return output;
}