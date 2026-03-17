#include "pure_pursuit.h"
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include "pid.h"
#include "planner.h"

#include <math.h>

// ============================================================
// Waypoints du circuit
// ============================================================

#define MAX_WAYPOINTS 2000

extern Point path[];
extern int pathLength;
static int currentWaypoint = 0;

// ============================================================
// PID direction
// ============================================================

static PID steeringPID;

// ============================================================
// Trouver waypoint lookahead
// ============================================================

static int findLookaheadIndex(float x, float y)
{
    for (int i = currentWaypoint; i < pathLength; i++)
    {
        float wx = path[i].x * 0.05f; // conversion grille → mètres
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
// Calcul angle Pure Pursuit
// ============================================================

static float computePurePursuitHeading(float x, float y, float theta)
{
    if(pathLength == 0)
        return 0;

    int idx = findLookaheadIndex(x, y);

    currentWaypoint = idx;

    //Value 0.05f à changer en fonction de la distance réelle
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
// Calcul moteurs avec PID
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
// Task FreeRTOS
// ============================================================

void purePursuitTask(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(3000));

    Serial.println("[NAV] Pure Pursuit task started");

    PID_init(&steeringPID, PP_KP, PP_KI, PP_KD, PP_DT);

    while (true)
    {
        if (currentNavMode != NAV_SLAM)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float x = robotPose.x;
        float y = robotPose.y;

        // conversion degrés → radians
        float theta = robotPose.theta * 0.0174532925f;

        float heading =
            computePurePursuitHeading(x, y, theta);

        int leftPWM = 0;
        int rightPWM = 0;

        computeMotorCommand(
            heading,
            &leftPWM,
            &rightPWM
        );

        // sécurité obstacle
        if(obstacleDetected)
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

        Serial.printf(
            "[PP] wp=%d h=%.2f L=%d R=%d\n",
            currentWaypoint,
            heading,
            leftPWM,
            rightPWM
        );

        vTaskDelay(pdMS_TO_TICKS(PP_TASK_PERIOD_MS));
    }
}