#include "naive_navigation.h"
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include <math.h>

// ============================================================
//  Carte angulaire locale (distance min par degré)
// ============================================================
static uint16_t distMap[360];

// ============================================================
//  Construction de la carte depuis le pointBuffer
// ============================================================
static void buildDistMap() {
  for (int i = 0; i < 360; i++) {
    distMap[i] = NAV_MAX_RANGE_MM;
  }

  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    unsigned long now = millis();

    for (int i = 0; i < pointsAvailable; i++) {
      int idx =
          (pointWriteIndex - 1 - i + POINT_BUFFER_SIZE) % POINT_BUFFER_SIZE;
      LidarPoint &p = pointBuffer[idx];

      if (now - p.ts > 500)
        continue;
      if (p.distance < NAV_MIN_RANGE_MM || p.distance > NAV_MAX_RANGE_MM)
        continue;
      if (p.confidence < 100)
        continue;

      int a = (int)p.angle;
      if (a < 0)
        a += 360;
      if (a >= 360)
        a -= 360;

      if (p.distance < distMap[a]) {
        distMap[a] = p.distance;
      }
    }
    xSemaphoreGive(bufferMutex);
  }
}

// ============================================================
//  Moyenne des distances adjacentes
// ============================================================
static uint16_t getAveragedDist(int angleDeg) {
  int a = angleDeg;
  if (a < 0)
    a += 360;
  if (a >= 360)
    a -= 360;

  uint32_t sum = 0;
  int count = 0;

  for (int offset = -NAV_AVG_WINDOW; offset <= NAV_AVG_WINDOW; offset++) {
    int idx = (a + offset + 360) % 360;
    sum += distMap[idx];
    count++;
  }

  return (uint16_t)(sum / count);
}

// ============================================================
//  Répulsion par les obstacles proches
// ============================================================
static float computeRepulsion() {
  float repulsion = 0.0f;

  for (int offset = -NAV_REPULSION_SCAN_ANGLE;
       offset <= NAV_REPULSION_SCAN_ANGLE; offset++) {
    if (offset == 0)
      continue;

    int mapIdx = offset;
    if (mapIdx < 0)
      mapIdx += 360;

    uint16_t dist = distMap[mapIdx];
    if (dist >= NAV_REPULSION_DIST_MM)
      continue;

    float strength = 1.0f - ((float)dist / (float)NAV_REPULSION_DIST_MM);

    if (offset > 0) {
      repulsion +=
          strength * NAV_REPULSION_GAIN; // Obstacle à droite → pousser à gauche
    } else {
      repulsion -=
          strength * NAV_REPULSION_GAIN; // Obstacle à gauche → pousser à droite
    }
  }

  return repulsion;
}

// ============================================================
//  Correction de dérive : centrage entre les murs
//
//  Compare la distance moyenne à gauche (90°) et à droite (270°)
//  et applique une correction pour rester centré.
// ============================================================
static float computeWallCentering() {
  // LD06 : 90° = DROITE du robot, 270° = GAUCHE du robot

  // Mesurer la distance au mur DROIT (autour de 90°)
  uint32_t rightSum = 0;
  int rightCount = 0;
  for (int a = 70; a <= 110; a++) {
    if (distMap[a] < NAV_MAX_RANGE_MM) {
      rightSum += distMap[a];
      rightCount++;
    }
  }

  // Mesurer la distance au mur GAUCHE (autour de 270°)
  uint32_t leftSum = 0;
  int leftCount = 0;
  for (int a = 250; a <= 290; a++) {
    if (distMap[a] < NAV_MAX_RANGE_MM) {
      leftSum += distMap[a];
      leftCount++;
    }
  }

  if (leftCount == 0 || rightCount == 0)
    return 0.0f;

  float leftDist = (float)leftSum / leftCount;
  float rightDist = (float)rightSum / rightCount;

  // Si plus proche du mur droit (rightDist < leftDist) → corriger vers gauche
  // (positif) Si plus proche du mur gauche (leftDist < rightDist) → corriger
  // vers droite (négatif)
  float diff =
      rightDist - leftDist; // positif = plus loin du mur droit = dérive gauche
  float correction = -diff * NAV_CENTERING_GAIN;

  // Limiter la correction
  if (correction > 8.0f)
    correction = 8.0f;
  if (correction < -8.0f)
    correction = -8.0f;

  return correction;
}

// ============================================================
//  computeNaiveHeading
// ============================================================
float computeNaiveHeading() {
  buildDistMap();

  float bestAngle = NAV_NO_PATH;
  uint16_t bestDist = 0;

  int halfAngle = (int)NAV_FORWARD_HALF_ANGLE;

  for (int step = 0; step <= halfAngle; step++) {
    int anglesToTest[2];
    int count;

    if (step == 0) {
      anglesToTest[0] = 0;
      count = 1;
    } else {
      anglesToTest[0] = step;
      anglesToTest[1] = -step;
      count = 2;
    }

    for (int s = 0; s < count; s++) {
      int offset = anglesToTest[s];
      int mapIndex = offset;
      if (mapIndex < 0)
        mapIndex += 360;

      uint16_t dist = getAveragedDist(mapIndex);

      if (dist < NAV_STOP_DISTANCE_MM)
        continue;

      if (dist > bestDist) {
        bestDist = dist;
        bestAngle = (float)offset;
      }
    }
  }

  // Ajouter répulsion + centrage entre murs
  if (bestAngle != NAV_NO_PATH) {
    float repulsion = computeRepulsion();
    float centering = computeWallCentering();
    bestAngle += repulsion + centering;
    bestAngle =
        constrain(bestAngle, -NAV_FORWARD_HALF_ANGLE, NAV_FORWARD_HALF_ANGLE);
  }

  return bestAngle;
}

// ============================================================
//  Vitesse adaptative
// ============================================================
static int computeAdaptiveSpeed(uint16_t bestDist) {
  if (bestDist >= NAV_CONFIDENCE_DIST_MM) {
    return NAV_FORWARD_SPEED;
  }
  float ratio = (float)(bestDist - NAV_STOP_DISTANCE_MM) /
                (float)(NAV_CONFIDENCE_DIST_MM - NAV_STOP_DISTANCE_MM);
  if (ratio < 0.0f)
    ratio = 0.0f;
  if (ratio > 1.0f)
    ratio = 1.0f;

  return NAV_SLOW_SPEED +
         (int)(ratio * (float)(NAV_FORWARD_SPEED - NAV_SLOW_SPEED));
}

// ============================================================
//  Tâche FreeRTOS
// ============================================================
void naiveNavigationTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println("[NAV] Naive navigation task started");

  bool wasEnabled = false;

  while (true) {
    // --- Vérifier activation via /naif ---
    if (!naifEnabled) {
      if (wasEnabled) {
        // Arrêter les moteurs ET remettre la commande à zéro
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

    if (heading == NAV_NO_PATH) {
      // BLOQUÉ : tourner à gauche sur place (pas d'arrêt !)
      // Tourne à gauche jusqu'à retrouver un cap à la prochaine itération
      leftPWM = -NAV_SPIN_PWM;
      rightPWM = NAV_SPIN_PWM;
      Serial.println("[NAV] No path -> spinning left");
    } else {
      // Récupérer distance pour vitesse adaptative
      int mapIdx = (int)heading;
      if (mapIdx < 0)
        mapIdx += 360;
      uint16_t bestDist = getAveragedDist(mapIdx);
      int speed = computeAdaptiveSpeed(bestDist);

      if (fabsf(heading) < 3.0f) {
        leftPWM = speed;
        rightPWM = speed;
      } else {
        float correction = heading * NAV_ANGLE_GAIN;
        correction = constrain(correction, -(float)speed, (float)speed);

        leftPWM = (int)(speed - correction);
        rightPWM = (int)(speed + correction);

        leftPWM = constrain(leftPWM, -255, 255);
        rightPWM = constrain(rightPWM, -255, 255);

        Serial.printf("[NAV] h=%.0f spd=%d L=%d R=%d\n", heading, speed,
                      leftPWM, rightPWM);
      }
    }

    // Écrire la commande moteur (toujours, même en spin)
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      motorCmd.leftPWM = leftPWM;
      motorCmd.rightPWM = rightPWM;
      motorCmd.timestamp = millis();
      xSemaphoreGive(ctrlMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
  }
}
