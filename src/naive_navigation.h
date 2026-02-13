#ifndef NAIVE_NAVIGATION_H
#define NAIVE_NAVIGATION_H

#include <Arduino.h>

// ============================================================
//  SUPERVARIABLES — Modifier ici pour ajuster le comportement
// ============================================================

// --- Dimensions Robot (mm) ---
#define NAV_ROBOT_WIDTH_MM 168.0f  // Largeur du robot
#define NAV_SAFETY_MARGIN_MM 50.0f // Marge latérale de sécurité
#define NAV_MIN_PASSAGE_MM (NAV_ROBOT_WIDTH_MM + 2.0f * NAV_SAFETY_MARGIN_MM)

// --- LiDAR ---
#define NAV_MAX_RANGE_MM 6000    // Portée max considérée (mm)
#define NAV_MIN_RANGE_MM 50      // En dessous = bruit
#define NAV_STOP_DISTANCE_MM 300 // Distance min avant stop complet

// --- Comportement ---
#define NAV_FORWARD_HALF_ANGLE 60.0f // Demi-cône de recherche avant (±60°)
#define NAV_FORWARD_SPEED 180        // PWM d'avance (0-255)
#define NAV_TURN_SPEED 160           // PWM de rotation sur place
#define NAV_ANGLE_GAIN 2.0f    // Gain proportionnel angle → correction moteur
#define NAV_TASK_PERIOD_MS 100 // Période de la boucle (10 Hz)

// --- Sentinel ---
#define NAV_NO_PATH -999

// ============================================================
//  API
// ============================================================

/**
 * @brief Calcule l'angle de cap optimal.
 *        0° = devant, positif = gauche, négatif = droite.
 *        Retourne NAV_NO_PATH si bloqué → tourne à gauche.
 */
float computeNaiveHeading();

/**
 * @brief Tâche FreeRTOS de navigation naïve.
 */
void naiveNavigationTask(void *pvParameters);

#endif
