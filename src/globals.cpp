#include "globals.h"
#include "speed_estimator.h"

SpeedEstimate estimatedSpeed = {0};

SemaphoreHandle_t ctrlMutex;
SemaphoreHandle_t lidarMutex;
SemaphoreHandle_t bufferMutex;
// QueueHandle_t packetQueue; // SUPPRIMÉ

ControlPacket ctrlData;
MotorCommand motorCmd = {0, 0, 0};
NavMode currentNavMode = NAV_NAIVE;
volatile int currentSpeedPWM = 0;
volatile int currentDirection = 0;
volatile bool naifEnabled = false;

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

uint32_t sentFeedback = 0;

RobotPose robotPose = {0.0f, 0.0f, 0.0f};
