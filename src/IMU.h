#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "QMI8658.h"
#include "AK09918.h"

typedef struct {
    float X;
    float Y;
    float Z;
} IMU_ST_SENSOR_DATA_FLOAT;

// Utilisation de EulerAngles directement (défini dans QMI8658.h)
extern EulerAngles stAngles; 
extern IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
extern IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
extern IMU_ST_SENSOR_DATA stMagnRawData;

void imuInit();
void imuDataGet(EulerAngles *pstAngles, 
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData);

void updateIMUData(); 
void calibrateMagn();

#endif