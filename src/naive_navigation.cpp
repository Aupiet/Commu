void naiveNavigationTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println("[NAV] Naive navigation task started");

  bool wasEnabled = false;

  while (true) {

    // ✅ UNIQUEMENT en mode NAIVE
    if (currentNavMode != NAV_NAIVE) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // --- Vérifier activation via /naif ---
    if (!naifEnabled) {
      if (wasEnabled) {
        stopMotors();
        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          motorCmd.leftPWM = 0;
          motorCmd.rightPWM = 0;
          motorCmd.timestamp = millis();
          xSemaphoreGive(ctrlMutex);
        }
        Serial.println("[NAV] Stopped (naif disabled)");
        wasEnabled = false;
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!wasEnabled) {
      Serial.println("[NAV] Started (naif enabled)");
      wasEnabled = true;
    }

    float heading = computeNaiveHeading();

    int leftPWM = 0;
    int rightPWM = 0;

    uint16_t frontCheck = getAveragedDist(0);
    bool frontBlocked = (frontCheck < NAV_STOP_DISTANCE_MM);

    if (heading == NAV_NO_PATH || frontBlocked) {
      leftPWM = -NAV_SPIN_PWM;
      rightPWM = NAV_SPIN_PWM;
    } else {
      uint16_t frontDist = getAveragedDist(0);
      int mapIdx = (int)heading;
      if (mapIdx < 0) mapIdx += 360;

      uint16_t headingDist = getAveragedDist(mapIdx);
      uint16_t minDist = (frontDist < headingDist) ? frontDist : headingDist;

      int speed = computeAdaptiveSpeed(minDist);

      float correction = heading * NAV_ANGLE_GAIN;
      correction = constrain(correction, -(float)speed, (float)speed);

      leftPWM = (int)(speed - correction);
      rightPWM = (int)(speed + correction);

      leftPWM = constrain(leftPWM, -255, 255);
      rightPWM = constrain(rightPWM, -255, 255);
    }

    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      motorCmd.leftPWM = leftPWM;
      motorCmd.rightPWM = rightPWM;
      motorCmd.timestamp = millis();
      xSemaphoreGive(ctrlMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
  }
}