#ifndef GLOBALS_H
#define GLOBALS_H

#include <WiFi.h>
#include "config.h"
//#include "speed_estimator.h"
// Ajouter dans la section "Control data" :
//extern SpeedEstimate estimatedSpeed;
// Synchronization
extern SemaphoreHandle_t ctrlMutex;
extern SemaphoreHandle_t lidarMutex;
extern SemaphoreHandle_t bufferMutex;
extern QueueHandle_t packetQueue;

// Control data
extern ControlPacket ctrlData;
extern volatile int currentSpeedPWM;
extern volatile int currentDirection;

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

// Stats
extern uint32_t receivedCommands;
extern uint32_t sentFeedback;

// Network
extern WiFiServer streamServer;
extern WiFiClient streamClient;

#endif