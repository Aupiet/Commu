#include "naive_navigation.h"
#include "config.h"
#include "globals.h"
#include <math.h>

// ============================================================
//  CARTE ANGULAIRE LOCALE (pas partagée — locale à la tâche)
// ============================================================
static uint16_t distMap[360]; // Distance min par degré (mm)

// ============================================================
//  Construction de la carte angulaire depuis le pointBuffer
// ============================================================
static void buildDistMap() {
  // Initialiser toutes les distances à MAX_RANGE (= pas d'obstacle connu)
  for (int i = 0; i < 360; i++) {
    distMap[i] = NAV_MAX_RANGE_MM;
  }

  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    unsigned long now = millis();

    for (int i = 0; i < pointsAvailable; i++) {
      // Lire en partant du dernier point écrit (les plus récents)
      int idx =
          (pointWriteIndex - 1 - i + POINT_BUFFER_SIZE) % POINT_BUFFER_SIZE;
      LidarPoint &p = pointBuffer[idx];

      // Ignorer les points trop vieux (> 500ms)
      if (now - p.ts > 500)
        continue;

      // Ignorer les mesures hors limites
      if (p.distance < NAV_MIN_RANGE_MM || p.distance > NAV_MAX_RANGE_MM)
        continue;
      if (p.confidence < 100)
        continue;

      int a = (int)p.angle;
      if (a < 0)
        a += 360;
      if (a >= 360)
        a -= 360;

      // Garder la distance MINIMALE (obstacle le plus proche à cet angle)
      if (p.distance < distMap[a]) {
        distMap[a] = p.distance;
      }
    }
    xSemaphoreGive(bufferMutex);
  }
}

// ============================================================
//  Vérification de la clearance latérale pour un angle donné
// ============================================================
static bool hasLateralClearance(int angleDeg) {
  // Normaliser l'angle dans [0, 360)
  int a = angleDeg;
  if (a < 0)
    a += 360;
  if (a >= 360)
    a -= 360;

  uint16_t frontDist = distMap[a];
  if (frontDist < NAV_MIN_RANGE_MM)
    return false;

  // Calculer l'angle sous-tendu par la demi-largeur du robot à cette distance
  // halfWidth / frontDist = tan(lateralAngle)
  float halfWidth = NAV_MIN_PASSAGE_MM / 2.0f;
  float lateralAngleDeg = atan2f(halfWidth, (float)frontDist) * (180.0f / M_PI);

  // Vérifier les angles latéraux (gauche et droite du cap candidat)
  int checkSpan = (int)ceilf(lateralAngleDeg);
  if (checkSpan < 1)
    checkSpan = 1;

  for (int offset = 1; offset <= checkSpan; offset++) {
    // Angle gauche
    int aLeft = (a + offset) % 360;
    // Angle droite
    int aRight = (a - offset + 360) % 360;

    // Distance minimale nécessaire sur le côté pour que le robot passe
    // À l'angle 'offset', la distance min = halfWidth / sin(offset_rad)
    float offsetRad = (float)offset * (M_PI / 180.0f);
    float sinVal = sinf(offsetRad);
    if (sinVal < 0.01f)
      continue; // Angle trop petit, pas significatif

    float minSideDist = halfWidth / sinVal;

    if (distMap[aLeft] < (uint16_t)minSideDist)
      return false;
    if (distMap[aRight] < (uint16_t)minSideDist)
      return false;
  }

  return true;
}

// ============================================================
//  computeNaiveHeading — Cœur de l'algorithme
// ============================================================
float computeNaiveHeading() {
  buildDistMap();

  float bestAngle = NAV_NO_PATH;
  uint16_t bestDist = 0;

  int halfAngle = (int)NAV_FORWARD_HALF_ANGLE;

  // Scanner depuis le CENTRE (0°) vers l'extérieur en alternant gauche/droite
  // Cela élimine le biais systématique vers la gauche
  for (int step = 0; step <= halfAngle; step++) {
    // Pour chaque step, on teste d'abord 0, puis +1/-1, +2/-2, etc.
    int sides = (step == 0) ? 1 : 2;
    for (int s = 0; s < sides; s++) {
      int offset = (s == 0) ? step : -step;

      int mapIndex = offset;
      if (mapIndex < 0)
        mapIndex += 360;

      uint16_t dist = distMap[mapIndex];

      if (dist < NAV_MIN_RANGE_MM)
        continue;

      if (!hasLateralClearance(mapIndex))
        continue;

      // Sélectionner la direction la plus éloignée,
      // mais préférer les angles proches de 0° (tout droit)
      if (dist > bestDist) {
        bestDist = dist;
        bestAngle = (float)offset;
      }

      // Si la distance est "assez bonne" (> 1.5m), prendre immédiatement
      // la première direction viable (la plus proche de tout droit)
      if (dist > 1500) {
        return (float)offset;
      }
    }
  }

  return bestAngle;
}

// ============================================================
//  Tâche FreeRTOS
// ============================================================

// Seuil minimum PWM pour que les moteurs tournent réellement
#define NAV_MIN_PWM 70

void naiveNavigationTask(void *pvParameters) {
  // Attendre que le système soit stable
  vTaskDelay(pdMS_TO_TICKS(2000));
  Serial.println("[NAV] Naive navigation task started");

  while (true) {
    // Vérifier le mode de navigation
    if (currentNavMode != NAV_NAIVE) {
      vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
      continue;
    }

    float heading = computeNaiveHeading();

    int leftPWM = 0;
    int rightPWM = 0;

    if (heading == NAV_NO_PATH) {
      // ===== AUCUN PASSAGE : rotation sur place à gauche =====
      leftPWM = -NAV_TURN_SPEED; // Gauche recule
      rightPWM = NAV_TURN_SPEED; // Droite avance
    } else if (fabsf(heading) < 3.0f) {
      // ===== TOUT DROIT =====
      leftPWM = NAV_FORWARD_SPEED;
      rightPWM = NAV_FORWARD_SPEED;
    } else {
      // ===== CORRECTION PROPORTIONNELLE =====
      float correction = heading * NAV_ANGLE_GAIN;
      correction = constrain(correction, -(float)NAV_FORWARD_SPEED,
                             (float)NAV_FORWARD_SPEED);

      leftPWM = (int)(NAV_FORWARD_SPEED - correction);
      rightPWM = (int)(NAV_FORWARD_SPEED + correction);

      // Clamp
      leftPWM = constrain(leftPWM, -255, 255);
      rightPWM = constrain(rightPWM, -255, 255);
    }

    // Appliquer le seuil MIN_PWM : si un moteur a une commande non-nulle
    // mais en dessous du seuil, forcer au minimum pour qu'il tourne
    if (leftPWM > 0 && leftPWM < NAV_MIN_PWM)
      leftPWM = NAV_MIN_PWM;
    if (leftPWM < 0 && leftPWM > -NAV_MIN_PWM)
      leftPWM = -NAV_MIN_PWM;
    if (rightPWM > 0 && rightPWM < NAV_MIN_PWM)
      rightPWM = NAV_MIN_PWM;
    if (rightPWM < 0 && rightPWM > -NAV_MIN_PWM)
      rightPWM = -NAV_MIN_PWM;

    // Écrire la commande moteur (protégé par mutex)
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      motorCmd.leftPWM = leftPWM;
      motorCmd.rightPWM = rightPWM;
      motorCmd.timestamp = millis();
      xSemaphoreGive(ctrlMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
  }
}
