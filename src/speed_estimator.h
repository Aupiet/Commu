#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <Arduino.h>

#define WHEEL_RADIUS_MM 50.0f          
#define WHEEL_CIRCUMFERENCE_MM (2.0f * 3.14159265f * WHEEL_RADIUS_MM)
#define ENCODER_PPR 20                 
#define ENCODER_DIST_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR) 

#define IMU_ACCEL_WEIGHT 0.3f          
#define ENCODER_WEIGHT 0.7f            
#define SPEED_FILTER_ALPHA 0.15f       
#define MAX_EXPECTED_SPEED 5000.0f     
#define MIN_ACCEL_MAGNITUDE 50.0f      

typedef struct {
    float speed_mm_per_sec;             
    float speed_m_per_sec;              
    float encoder_speed_mm_per_sec;     
    float imu_speed_mm_per_sec;         
    float confidence;                   
    unsigned long last_update_ms;       
} SpeedEstimate;

void initSpeedEstimator();
void onEncoderLeftPulse();
void onEncoderRightPulse();
void speedEstimatorTask(void *pvParameters);
SpeedEstimate getSpeedEstimate();
void resetSpeedEstimator();

// Déclaration externe (la définition est dans globals.cpp)
extern SpeedEstimate estimatedSpeed; 

#endif