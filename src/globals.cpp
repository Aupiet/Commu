#include "globals.h"
#include "speed_estimator.h"

// --- CORRECTION : C'est ici qu'on crée la variable en mémoire ---
SpeedEstimate estimatedSpeed = {0};

uint8_t esp01Address[] = {0x34, 0x5F, 0x45, 0x62, 0xAC, 0x9C};

SemaphoreHandle_t ctrlMutex;
SemaphoreHandle_t lidarMutex;
SemaphoreHandle_t bufferMutex;
QueueHandle_t packetQueue;

ControlPacket ctrlData;
volatile int currentSpeedPWM = 0;
volatile int currentDirection = 0;

LidarPoint pointBuffer[POINT_BUFFER_SIZE];
int pointWriteIndex = 0;
int pointReadIndex = 0;
volatile int pointsAvailable = 0;
volatile bool obstacleDetected = false;
volatile uint16_t minObstacleDistance = 9999;
volatile unsigned long packetsReceived = 0;
volatile unsigned long totalPointsProcessed = 0;
volatile float currentRotationSpeed = 0.0f;

unsigned long lastCommandTime = 0;
unsigned long lastFeedbackTime = 0;
unsigned long lastLidarUpdate = 0;
unsigned long lastOledUpdate = 0;

uint32_t receivedCommands = 0;
uint32_t sentFeedback = 0;

WiFiServer streamServer(STREAM_PORT);
WiFiClient streamClient;