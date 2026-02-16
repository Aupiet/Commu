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
//
//  Au lieu de prendre la distance brute d'un seul degré,
//  on fait la moyenne sur ±NAV_AVG_WINDOW degrés pour obtenir
//  une estimation plus robuste de la distance dans une
//  direction donnée.
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
//  Vérification clearance latérale
// ============================================================
static bool hasLateralClearance(int angleDeg, uint16_t frontDist) {
  int a = angleDeg;
  if (a < 0)
    a += 360;
  if (a >= 360)
    a -= 360;

  if (frontDist < NAV_STOP_DISTANCE_MM)
    return false;

  float halfWidth = NAV_MIN_PASSAGE_MM / 2.0f; // 104 mm (réduit)

  // Vérifier les angles adjacents (±1° à ±10°) — réduit de 15
  int maxCheck = 10;

  for (int offset = 1; offset <= maxCheck; offset++) {
    int aLeft = (a + offset) % 360;
    int aRight = (a - offset + 360) % 360;

    float offsetRad = (float)offset * (M_PI / 180.0f);
    float sinVal = sinf(offsetRad);

    float perpLeft = (float)distMap[aLeft] * sinVal;
    float perpRight = (float)distMap[aRight] * sinVal;

    float obstFrontLeft = (float)distMap[aLeft] * cosf(offsetRad);
    float obstFrontRight = (float)distMap[aRight] * cosf(offsetRad);

    if (obstFrontLeft > 0 && obstFrontLeft < (float)frontDist) {
      if (perpLeft < halfWidth)
        return false;
    }
    if (obstFrontRight > 0 && obstFrontRight < (float)frontDist) {
      if (perpRight < halfWidth)
        return false;
    }
  }

  return true;
}

// ============================================================
//  computeNaiveHeading — Algorithme principal
//
//  Utilise getAveragedDist() au lieu de distMap[] brut
//  pour une sélection de direction plus stable.
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

      // Utiliser la distance moyennée sur les points adjacents
      uint16_t dist = getAveragedDist(mapIndex);

      if (dist < NAV_STOP_DISTANCE_MM)
        continue;

      if (!hasLateralClearance(mapIndex, dist))
        continue;

      if (dist > bestDist) {
        bestDist = dist;
        bestAngle = (float)offset;
      }
    }
  }

  return bestAngle;
}

// ============================================================
//  Calcul de la vitesse adaptative
//
//  Plus la meilleure distance est courte, plus on ralentit.
//  Au-delà de NAV_CONFIDENCE_DIST_MM → pleine vitesse.
// ============================================================
static int computeAdaptiveSpeed(uint16_t bestDist) {
  if (bestDist >= NAV_CONFIDENCE_DIST_MM) {
    return NAV_FORWARD_SPEED;
  }
  // Interpolation linéaire entre SLOW et FORWARD
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

  // Lissage EMA du heading
  float smoothedHeading = 0.0f;
  bool firstHeading = true;

  // Machine à états pour le spin (dernier recours)
  enum NavState { NAV_RUNNING, NAV_SPINNING };
  NavState state = NAV_RUNNING;
  unsigned long spinStartTime = 0;

  while (true) {
    // --- Vérifier si l'algorithme naïf est activé via /naif ---
    if (!naifEnabled) {
      if (wasEnabled) {
        stopMotors();
        Serial.println("[NAV] Stopped (naif disabled)");
        wasEnabled = false;
        state = NAV_RUNNING;
        firstHeading = true;
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!wasEnabled) {
      Serial.println("[NAV] Started (naif enabled)");
      wasEnabled = true;
      firstHeading = true;
      smoothedHeading = 0.0f;
      state = NAV_RUNNING;
    }

    int leftPWM = 0;
    int rightPWM = 0;

    // ========================================================
    //  ÉTAT: SPINNING — rotation en dernier recours
    //  Non-bloquant : on vérifie si le temps est écoulé
    // ========================================================
    if (state == NAV_SPINNING) {
      if (millis() - spinStartTime < NAV_SPIN_DURATION_MS) {
        // Continuer à tourner à gauche
        leftPWM = -NAV_SPIN_PWM;
        rightPWM = NAV_SPIN_PWM;
      } else {
        // Spin terminé → retour à NAV_RUNNING
        state = NAV_RUNNING;
        firstHeading = true;
        Serial.println("[NAV] Spin done, resuming");
        // On ne stoppe pas : on passe directement à l'analyse ci-dessous
        // au prochain tick
      }

      // Écrire la commande moteur (spin ou arrêt)
      if (state == NAV_SPINNING) {
        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          motorCmd.leftPWM = leftPWM;
          motorCmd.rightPWM = rightPWM;
          motorCmd.timestamp = millis();
          xSemaphoreGive(ctrlMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
        continue;
      }
    }

    // ========================================================
    //  ÉTAT: RUNNING — navigation normale
    // ========================================================
    float heading = computeNaiveHeading();

    if (heading == NAV_NO_PATH) {
      // BLOQUÉ → passer en spin (dernier recours, non-bloquant)
      Serial.println("[NAV] BLOCKED -> spin left (last resort)");
      state = NAV_SPINNING;
      spinStartTime = millis();
      vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
      continue;
    }

    // --- Lissage EMA de la direction ---
    if (firstHeading) {
      smoothedHeading = heading;
      firstHeading = false;
    } else {
      smoothedHeading = NAV_SMOOTHING_ALPHA * heading +
                        (1.0f - NAV_SMOOTHING_ALPHA) * smoothedHeading;
    }

    // --- Récupérer la meilleure distance pour la vitesse adaptative ---
    int mapIdx = (int)smoothedHeading;
    if (mapIdx < 0)
      mapIdx += 360;
    uint16_t bestDist = getAveragedDist(mapIdx);
    int speed = computeAdaptiveSpeed(bestDist);

    if (fabsf(smoothedHeading) < 3.0f) {
      // TOUT DROIT
      leftPWM = speed;
      rightPWM = speed;
    } else {
      // CORRECTION PROPORTIONNELLE
      float correction = smoothedHeading * NAV_ANGLE_GAIN;
      correction = constrain(correction, -(float)speed, (float)speed);

      leftPWM = (int)(speed - correction);
      rightPWM = (int)(speed + correction);

      leftPWM = constrain(leftPWM, -255, 255);
      rightPWM = constrain(rightPWM, -255, 255);

      Serial.printf("[NAV] h=%.0f sh=%.0f spd=%d L=%d R=%d\n", heading,
                    smoothedHeading, speed, leftPWM, rightPWM);
    }

    // Écrire la commande moteur
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      motorCmd.leftPWM = leftPWM;
      motorCmd.rightPWM = rightPWM;
      motorCmd.timestamp = millis();
      xSemaphoreGive(ctrlMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
  }
}
