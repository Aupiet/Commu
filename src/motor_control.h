#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

void initMotors();
void stopMotors();
void testMotors();
void channelACtrl(int pwmInput);
void channelBCtrl(int pwmInput);
void setMotorSpeedFromJoystick(int joyX, int joyY);
void motorControlTask(void *pvParameters);

#endif