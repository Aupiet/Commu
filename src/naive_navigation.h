#ifndef NAIVE_NAVIGATION_H
#define NAIVE_NAVIGATION_H

#include <Arduino.h>

// ============================================================
//  SUPERVARIABLES — Modifier ici pour ajuster le comportement
// ============================================================

// --- Dimensions Robot (mm) ---
#define NAV_ROBOT_WIDTH_MM 168.0f
#define NAV_SAFETY_MARGIN_MM 20.0f
#define NAV_MIN_PASSAGE_MM (NAV_ROBOT_WIDTH_MM + 2.0f * NAV_SAFETY_MARGIN_MM)

// --- LiDAR ---
#define NAV_MAX_RANGE_MM 6000
#define NAV_MIN_RANGE_MM 50
#define NAV_STOP_DISTANCE_MM 200

// --- Comportement ---
#define NAV_FORWARD_HALF_ANGLE 60.0f
#define NAV_FORWARD_SPEED 180 // -30%
#define NAV_SLOW_SPEED 70     // -30%
#define NAV_ANGLE_GAIN 2.0f
#define NAV_TASK_PERIOD_MS 100
#define NAV_AVG_WINDOW 5

// --- Répulsion obstacles ---
#define NAV_REPULSION_DIST_MM 800 // Distance sous laquelle un obstacle repousse
#define NAV_REPULSION_GAIN 20.0f  // Force de la répulsion (degrés)
#define NAV_REPULSION_SCAN_ANGLE                                               \
  90 // Demi-angle de scan pour la répulsion (±90°)

// --- Centrage entre murs (anti-dérive) ---
#define NAV_CENTERING_GAIN                                                     \
  0.03f // Force du centrage (degrés par mm de différence)

// --- Rotation quand bloqué ---
#define NAV_SPIN_PWM 120

// --- Vitesse adaptative ---
#define NAV_CONFIDENCE_DIST_MM 1500

// --- Sentinel ---
#define NAV_NO_PATH -999

// ============================================================
//  API
// ============================================================

float computeNaiveHeading();
void naiveNavigationTask(void *pvParameters);

#endif
