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
//
//  Scanne ±NAV_REPULSION_SCAN_ANGLE autour de l'avant.
//  Chaque obstacle proche pousse le robot dans la direction
//  opposée, proportionnellement à sa proximité et son angle.
//
//  Retourne un angle de correction en degrés :
//    positif = tourner à gauche, négatif = tourner à droite.
// ============================================================
static float computeRepulsion() {
  float repulsion = 0.0f;

  for (int offset = -NAV_REPULSION_SCAN_ANGLE;
       offset <= NAV_REPULSION_SCAN_ANGLE; offset++) {
    if (offset == 0)
      continue; // Ignorer droit devant (géré par bestAngle)

    int mapIdx = offset;
    if (mapIdx < 0)
      mapIdx += 360;

    uint16_t dist = distMap[mapIdx];

    if (dist >= NAV_REPULSION_DIST_MM)
      continue; // Trop loin, pas de répulsion

    // Force inversement proportionnelle à la distance
    // Plus c'est proche, plus ça repousse
    float strength = 1.0f - ((float)dist / (float)NAV_REPULSION_DIST_MM);
    // strength ∈ [0, 1] : 0 = loin, 1 = très proche

    // Pousser dans la direction OPPOSÉE à l'obstacle
    // Obstacle à gauche (offset > 0) → repousse à droite (négatif)
    // Obstacle à droite (offset < 0) → repousse à gauche (positif)
    if (offset > 0) {
      repulsion -= strength * NAV_REPULSION_GAIN; // Pousser à droite
    } else {
      repulsion += strength * NAV_REPULSION_GAIN; // Pousser à gauche
    }
  }

  return repulsion;
}

// ============================================================
//  computeNaiveHeading — Algorithme principal
//
//  1. Trouver le point le plus éloigné dans le cône avant
//     (utilise distances moyennées, PAS de clearance latérale)
//  2. Ajouter la répulsion des obstacles proches
//  3. Retourne l'angle final
// ============================================================
float computeNaiveHeading() {
  buildDistMap();

  float bestAngle = NAV_NO_PATH;
  uint16_t bestDist = 0;

  int halfAngle = (int)NAV_FORWARD_HALF_ANGLE;

  // Scanner du centre vers l'extérieur, GAUCHE d'abord
  for (int step = 0; step <= halfAngle; step++) {
    int anglesToTest[2];
    int count;

    if (step == 0) {
      anglesToTest[0] = 0;
      count = 1;
    } else {
      anglesToTest[0] = step;  // Gauche d'abord
      anglesToTest[1] = -step; // Droite ensuite
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

  // Si on a trouvé un chemin, ajouter la répulsion
  if (bestAngle != NAV_NO_PATH) {
    float repulsion = computeRepulsion();
    bestAngle += repulsion;
    // Contraindre dans le cône
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
//  Tâche FreeRTOS — Machine à états non-bloquante
// ============================================================
void naiveNavigationTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println("[NAV] Naive navigation task started");

  bool wasEnabled = false;

  // Machine à états pour le spin (dernier recours)
  enum NavState { NAV_RUNNING, NAV_SPINNING };
  NavState state = NAV_RUNNING;
  unsigned long spinStartTime = 0;

  while (true) {
    // --- Vérifier activation via /naif ---
    if (!naifEnabled) {
      if (wasEnabled) {
        stopMotors();
        Serial.println("[NAV] Stopped (naif disabled)");
        wasEnabled = false;
        state = NAV_RUNNING;
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!wasEnabled) {
      Serial.println("[NAV] Started (naif enabled)");
      wasEnabled = true;
      state = NAV_RUNNING;
    }

    int leftPWM = 0;
    int rightPWM = 0;

    // ========================================================
    //  SPINNING — dernier recours, non-bloquant
    // ========================================================
    if (state == NAV_SPINNING) {
      if (millis() - spinStartTime < NAV_SPIN_DURATION_MS) {
        leftPWM = -NAV_SPIN_PWM;
        rightPWM = NAV_SPIN_PWM;

        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          motorCmd.leftPWM = leftPWM;
          motorCmd.rightPWM = rightPWM;
          motorCmd.timestamp = millis();
          xSemaphoreGive(ctrlMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
        continue;
      } else {
        state = NAV_RUNNING;
        Serial.println("[NAV] Spin done, resuming");
      }
    }

    // ========================================================
    //  RUNNING — navigation réactive
    // ========================================================
    float heading = computeNaiveHeading();

    if (heading == NAV_NO_PATH) {
      Serial.println("[NAV] BLOCKED -> spin left");
      state = NAV_SPINNING;
      spinStartTime = millis();
      vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
      continue;
    }

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

      Serial.printf("[NAV] h=%.0f spd=%d L=%d R=%d\n", heading, speed, leftPWM,
                    rightPWM);
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
