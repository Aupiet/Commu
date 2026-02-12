#include "naive_navigation.h"
#include "globals.h"
#include "lidar_manager.h"
#include "motor_control.h"

void naiveNavigationTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(NAV_TASK_PERIOD_MS);

  for (;;) {
    // Si obstacle détecté (flag global mis à jour par lidarTask)
    if (obstacleDetected) {
      stopMotors();
    } else {
      // Pas d'obstacle : on avance tout droit
      channelACtrl(NAV_FORWARD_SPEED);
      channelBCtrl(NAV_FORWARD_SPEED);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

float computeNaiveHeading() {
  return 0.0f; // Not used in this simplified version
}
