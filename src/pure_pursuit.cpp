#include "pure_pursuit.h"
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include "pid.h"
#include "planner.h"

#include <math.h>

#define MAX_WAYPOINTS 2000

extern Point path[];
extern int pathLength;
static int currentWaypoint = 0;

static PID steeringPID;

// ============================================================
// Lookahead
// ============================================================
static int findLookaheadIndex(float x, float y)
{
    for (int i = currentWaypoint; i < pathLength; i++)
    {
        float wx = path[i].x * 0.05f;
        float wy = path[i].y * 0.05f;

        float dx = wx - x;
        float dy = wy - y;

        float dist = sqrtf(dx * dx + dy * dy);

        if (dist >= PP_LOOKAHEAD_DIST)
            return i;
    }

    return pathLength - 1;
}

// ============================================================
// Heading
// ============================================================
static float computePurePursuitHeading(float x, float y, float theta)
{
    if (pathLength == 0)
        return 0;

    int idx = findLookaheadIndex(x, y);
    currentWaypoint = idx;

    float wx = path[idx].x * 0.05f;
    float wy = path[idx].y * 0.05f;

    float dx = wx - x;
    float dy = wy - y;

    float localX = cosf(theta) * dx + sinf(theta) * dy;
    float localY = -sinf(theta) * dx + cosf(theta) * dy;

    float curvature =
        (2.0f * localY) /
        (PP_LOOKAHEAD_DIST * PP_LOOKAHEAD_DIST);

    float heading = atanf(PP_WHEELBASE * curvature);

    return heading;
}

// ============================================================
// PID moteurs
// ============================================================
static void computeMotorCommand(float heading, int *leftPWM, int *rightPWM)
{
    int baseSpeed = PP_BASE_SPEED;

    float correction = PID_update(&steeringPID, 0.0f, heading);

    int left = baseSpeed - (int)(correction * PP_STEERING_GAIN);
    int right = baseSpeed + (int)(correction * PP_STEERING_GAIN);

    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    *leftPWM = left;
    *rightPWM = right;
}

// ============================================================
// TASK
// ============================================================
void purePursuitTask(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(3000));

    Serial.println("[NAV] Pure Pursuit task started");

    PID_init(&steeringPID, PP_KP, PP_KI, PP_KD, PP_DT);

    while (true)
    {
        // ✅ UNIQUEMENT mode autonome
        if (currentNavMode != NAV_AUTONOMOUS)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // ✅ Sécurité : pas de chemin
        if (pathLength == 0)
        {
            Serial.println("[PP] No path");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float x = robotPose.x;
        float y = robotPose.y;
        float theta = robotPose.theta * 0.0174532925f;

        float heading =
            computePurePursuitHeading(x, y, theta);

        int leftPWM = 0;
        int rightPWM = 0;

        computeMotorCommand(heading, &leftPWM, &rightPWM);

        // Sécurité obstacle
        if (obstacleDetected)
        {
            leftPWM = -120;
            rightPWM = 120;
        }

        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            motorCmd.leftPWM = leftPWM;
            motorCmd.rightPWM = rightPWM;
            motorCmd.timestamp = millis();
            xSemaphoreGive(ctrlMutex);
        }

        Serial.printf("[PP] wp=%d h=%.2f L=%d R=%d\n",
                      currentWaypoint, heading, leftPWM, rightPWM);

        vTaskDelay(pdMS_TO_TICKS(PP_TASK_PERIOD_MS));
    }
}