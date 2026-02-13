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
  // Init : tout à max range (= pas d'obstacle connu)
  for (int i = 0; i < 360; i++) {
    distMap[i] = NAV_MAX_RANGE_MM;
  }

  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    unsigned long now = millis();

    for (int i = 0; i < pointsAvailable; i++) {
      int idx =
          (pointWriteIndex - 1 - i + POINT_BUFFER_SIZE) % POINT_BUFFER_SIZE;
      LidarPoint &p = pointBuffer[idx];

      // Ignorer les points trop vieux (> 500ms)
      if (now - p.ts > 500)
        continue;

      // Filtres qualité
      if (p.distance < NAV_MIN_RANGE_MM || p.distance > NAV_MAX_RANGE_MM)
        continue;
      if (p.confidence < 100)
        continue;

      int a = (int)p.angle;
      if (a < 0)
        a += 360;
      if (a >= 360)
        a -= 360;

      // Garder la distance MINIMALE (obstacle le plus proche)
      if (p.distance < distMap[a]) {
        distMap[a] = p.distance;
      }
    }
    xSemaphoreGive(bufferMutex);
  }
}

// ============================================================
//  Vérification clearance latérale pour un angle donné
//  Le robot a une certaine largeur : on vérifie que les côtés
//  sont aussi dégagés pour qu'il puisse passer.
// ============================================================
static bool hasLateralClearance(int angleDeg) {
  int a = angleDeg;
  if (a < 0)
    a += 360;
  if (a >= 360)
    a -= 360;

  uint16_t frontDist = distMap[a];
  if (frontDist < NAV_MIN_RANGE_MM)
    return false;

  // Angle sous-tendu par la demi-largeur du robot à cette distance
  float halfWidth = NAV_MIN_PASSAGE_MM / 2.0f;
  float lateralAngleDeg = atan2f(halfWidth, (float)frontDist) * (180.0f / M_PI);

  int checkSpan = (int)ceilf(lateralAngleDeg);
  if (checkSpan < 1)
    checkSpan = 1;

  for (int offset = 1; offset <= checkSpan; offset++) {
    int aLeft = (a + offset) % 360;
    int aRight = (a - offset + 360) % 360;

    // Distance min nécessaire sur le côté
    float offsetRad = (float)offset * (M_PI / 180.0f);
    float sinVal = sinf(offsetRad);
    if (sinVal < 0.01f)
      continue;

    float minSideDist = halfWidth / sinVal;

    if (distMap[aLeft] < (uint16_t)minSideDist)
      return false;
    if (distMap[aRight] < (uint16_t)minSideDist)
      return false;
  }

  return true;
}

// ============================================================
//  computeNaiveHeading — Algorithme principal
//
//  Règles :
//  1. Chercher dans le cône avant (±NAV_FORWARD_HALF_ANGLE)
//  2. Choisir la direction avec le point le plus ÉLOIGNÉ
//  3. Si deux points sont à la même distance → préférer GAUCHE
//  4. Vérifier la clearance latérale (largeur robot)
//  5. Si aucun chemin viable → retourner NAV_NO_PATH
// ============================================================
float computeNaiveHeading() {
  buildDistMap();

  float bestAngle = NAV_NO_PATH;
  uint16_t bestDist = 0;

  int halfAngle = (int)NAV_FORWARD_HALF_ANGLE;

  // Scanner depuis le centre vers l'extérieur
  // Pour chaque step, tester GAUCHE d'abord (priorité gauche)
  for (int step = 0; step <= halfAngle; step++) {
    // Angles à tester : 0, puis +1, -1, +2, -2, ...
    // Mais avec priorité gauche : on teste +step AVANT -step
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

      uint16_t dist = distMap[mapIndex];

      // Filtrer les distances trop courtes
      if (dist < NAV_STOP_DISTANCE_MM)
        continue;

      // Vérifier que le robot peut physiquement passer
      if (!hasLateralClearance(mapIndex))
        continue;

      // Sélectionner la direction la plus éloignée
      // En cas d'égalité : la gauche gagne car testée en premier
      if (dist > bestDist) {
        bestDist = dist;
        bestAngle = (float)offset;
      }

      // Si distance > 1.5m et viable, prendre immédiatement
      // (la plus proche de tout droit, grâce au scan centre→extérieur)
      if (dist > 1500) {
        return (float)offset;
      }
    }
  }

  return bestAngle;
}

// ============================================================
//  Tâche FreeRTOS — Pilotage moteurs depuis le heading calculé
// ============================================================

#define NAV_MIN_PWM 70 // Seuil minimum pour que les moteurs tournent

void naiveNavigationTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(3000)); // Attendre que le système soit stable
  Serial.println("[NAV] Naive navigation task started");

  while (true) {
    float heading = computeNaiveHeading();

    int leftPWM = 0;
    int rightPWM = 0;

    if (heading == NAV_NO_PATH) {
      // ===== BLOQUÉ : rotation sur place à GAUCHE =====
      leftPWM = -NAV_TURN_SPEED;
      rightPWM = NAV_TURN_SPEED;
      Serial.println("[NAV] BLOCKED → turning left");
    } else if (fabsf(heading) < 3.0f) {
      // ===== TOUT DROIT =====
      leftPWM = NAV_FORWARD_SPEED;
      rightPWM = NAV_FORWARD_SPEED;
    } else {
      // ===== CORRECTION PROPORTIONNELLE =====
      // heading > 0 → tourner à gauche (ralentir gauche)
      // heading < 0 → tourner à droite (ralentir droite)
      float correction = heading * NAV_ANGLE_GAIN;
      correction = constrain(correction, -(float)NAV_FORWARD_SPEED,
                             (float)NAV_FORWARD_SPEED);

      leftPWM = (int)(NAV_FORWARD_SPEED - correction);
      rightPWM = (int)(NAV_FORWARD_SPEED + correction);

      leftPWM = constrain(leftPWM, -255, 255);
      rightPWM = constrain(rightPWM, -255, 255);
    }

    // Appliquer seuil MIN_PWM
    if (leftPWM > 0 && leftPWM < NAV_MIN_PWM)
      leftPWM = NAV_MIN_PWM;
    if (leftPWM < 0 && leftPWM > -NAV_MIN_PWM)
      leftPWM = -NAV_MIN_PWM;
    if (rightPWM > 0 && rightPWM < NAV_MIN_PWM)
      rightPWM = NAV_MIN_PWM;
    if (rightPWM < 0 && rightPWM > -NAV_MIN_PWM)
      rightPWM = -NAV_MIN_PWM;

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
