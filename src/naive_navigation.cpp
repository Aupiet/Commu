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
//  Vérification clearance latérale
//
//  Pour un angle donné, on vérifie que les angles adjacents
//  (± quelques degrés) n'ont pas d'obstacles trop proches.
//  On calcule la distance perpendiculaire entre la trajectoire
//  robot et l'obstacle latéral. Si cette distance > demi-largeur
//  robot → OK.
// ============================================================
static bool hasLateralClearance(int angleDeg, uint16_t frontDist) {
  int a = angleDeg;
  if (a < 0)
    a += 360;
  if (a >= 360)
    a -= 360;

  if (frontDist < NAV_STOP_DISTANCE_MM)
    return false;

  float halfWidth = NAV_MIN_PASSAGE_MM / 2.0f; // 134 mm

  // Vérifier les angles adjacents (±1° à ±15°)
  // Pour chaque angle latéral, l'obstacle est à distMap[a±offset]
  // La distance perpendiculaire = dist_laterale * sin(offset)
  // Si dist_perp < halfWidth → le robot ne passe pas
  int maxCheck = 15;

  for (int offset = 1; offset <= maxCheck; offset++) {
    int aLeft = (a + offset) % 360;
    int aRight = (a - offset + 360) % 360;

    float offsetRad = (float)offset * (M_PI / 180.0f);
    float sinVal = sinf(offsetRad);

    // Distance perpendiculaire de l'obstacle à la trajectoire
    float perpLeft = (float)distMap[aLeft] * sinVal;
    float perpRight = (float)distMap[aRight] * sinVal;

    // On ne vérifie que si l'obstacle est PLUS PROCHE que notre trajectoire
    // (i.e. si l'obstacle est en travers de notre chemin)
    float obstFrontLeft = (float)distMap[aLeft] * cosf(offsetRad);
    float obstFrontRight = (float)distMap[aRight] * cosf(offsetRad);

    // L'obstacle latéral n'est un problème que s'il est DEVANT nous
    // (sa projection sur notre axe < notre distance de trajet)
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
//  1. Chercher dans le cône avant (±NAV_FORWARD_HALF_ANGLE)
//  2. Choisir la direction avec le point le plus ÉLOIGNÉ
//  3. Équidistance → préférer GAUCHE
//  4. Vérifier clearance latérale (largeur robot)
//  5. Si bloqué → NAV_NO_PATH
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

      uint16_t dist = distMap[mapIndex];

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
//  Tâche FreeRTOS
// ============================================================
void naiveNavigationTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println("[NAV] Naive navigation task started");

  while (true) {
    float heading = computeNaiveHeading();

    int leftPWM = 0;
    int rightPWM = 0;

    if (heading == NAV_NO_PATH) {
      // BLOQUÉ : rotation sur place à gauche
      leftPWM = -NAV_TURN_SPEED;
      rightPWM = NAV_TURN_SPEED;
      Serial.println("[NAV] BLOCKED -> turn left");
    } else if (fabsf(heading) < 3.0f) {
      // TOUT DROIT
      leftPWM = NAV_FORWARD_SPEED;
      rightPWM = NAV_FORWARD_SPEED;
    } else {
      // CORRECTION PROPORTIONNELLE
      float correction = heading * NAV_ANGLE_GAIN;
      correction = constrain(correction, -(float)NAV_FORWARD_SPEED,
                             (float)NAV_FORWARD_SPEED);

      leftPWM = (int)(NAV_FORWARD_SPEED - correction);
      rightPWM = (int)(NAV_FORWARD_SPEED + correction);

      leftPWM = constrain(leftPWM, -255, 255);
      rightPWM = constrain(rightPWM, -255, 255);

      Serial.printf("[NAV] heading=%.0f L=%d R=%d\n", heading, leftPWM,
                    rightPWM);
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
