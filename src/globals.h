#ifndef GLOBALS_H
#define GLOBALS_H

#include "config.h"

// #include "speed_estimator.h"
//  Ajouter dans la section "Control data" :
// extern SpeedEstimate estimatedSpeed;
//  Synchronization
extern SemaphoreHandle_t ctrlMutex;
extern SemaphoreHandle_t lidarMutex;
extern SemaphoreHandle_t bufferMutex;
extern QueueHandle_t packetQueue;

// Control data
extern ControlPacket ctrlData;
extern MotorCommand motorCmd;
extern NavMode currentNavMode;
extern volatile int currentSpeedPWM;
extern volatile int currentDirection;
extern volatile bool naifEnabled;

// LiDAR data
extern LidarPoint pointBuffer[POINT_BUFFER_SIZE];
extern int pointWriteIndex;
extern int pointReadIndex;
extern volatile int pointsAvailable;
extern volatile bool obstacleDetected;
extern volatile uint16_t minObstacleDistance;
extern volatile unsigned long packetsReceived;
extern volatile unsigned long totalPointsProcessed;
extern volatile float currentRotationSpeed;

// Timings
extern unsigned long lastCommandTime;
extern unsigned long lastFeedbackTime;
extern unsigned long lastLidarUpdate;
extern unsigned long lastOledUpdate;

extern uint32_t sentFeedback;

#endif