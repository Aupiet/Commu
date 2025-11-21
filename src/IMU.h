#ifndef IMU_H
#define IMU_H

#include "QMI8658.h"
#include "AK09918.h"
#include <Arduino.h>

// Ne pas redéclarer IMU_ST_SENSOR_DATA ici, utilisez celui de AK09918.h !

typedef struct {
    float fYaw;
    float fPitch;
    float fRoll;
} IMU_ST_ANGLES_DATA;

typedef struct {
    float X;
    float Y;
    float Z;
} IMU_ST_SENSOR_DATA_FLOAT;

void imuInit();
void updateIMUData();
float getIMUSpeed(float dt);
void sendSpeedToATMEGA(float speed);

// Utilise les types AK09918.h pour IMU_ST_SENSOR_DATA
extern IMU_ST_ANGLES_DATA stAngles;
extern IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
extern IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
extern IMU_ST_SENSOR_DATA stMagnRawData; // Ce type existe via AK09918.h

#endif
