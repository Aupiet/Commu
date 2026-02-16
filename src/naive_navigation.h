#ifndef NAIVE_NAVIGATION_H
#define NAIVE_NAVIGATION_H

#include <Arduino.h>

// ============================================================
//  SUPERVARIABLES — Modifier ici pour ajuster le comportement
// ============================================================

// --- Dimensions Robot (mm) ---
#define NAV_ROBOT_WIDTH_MM 168.0f  // Largeur du robot
#define NAV_SAFETY_MARGIN_MM 20.0f // Marge latérale de sécurité (réduite)
#define NAV_MIN_PASSAGE_MM (NAV_ROBOT_WIDTH_MM + 2.0f * NAV_SAFETY_MARGIN_MM)

// --- LiDAR ---
#define NAV_MAX_RANGE_MM 6000    // Portée max considérée (mm)
#define NAV_MIN_RANGE_MM 50      // En dessous = bruit
#define NAV_STOP_DISTANCE_MM 200 // Distance min avant stop complet (abaissée)

// --- Comportement ---
#define NAV_FORWARD_HALF_ANGLE 60.0f // Demi-cône de recherche avant (±60°)
#define NAV_FORWARD_SPEED 180        // PWM max d'avance (0-255)
#define NAV_SLOW_SPEED 100           // PWM réduit quand incertain
#define NAV_TURN_SPEED 160           // PWM de rotation sur place
#define NAV_ANGLE_GAIN 2.0f    // Gain proportionnel angle → correction moteur
#define NAV_TASK_PERIOD_MS 100 // Période de la boucle (10 Hz)

// --- Lissage direction (EMA) ---
#define NAV_SMOOTHING_ALPHA                                                    \
  0.3f                   // Poids nouvelle mesure (0.0=inerte, 1.0=réactif)
#define NAV_AVG_WINDOW 5 // Demi-fenêtre pour moyenner les distances adjacentes

// --- Rotation bloqué (non-bloquante) ---
#define NAV_SPIN_DURATION_MS 400 // Durée du spin en dernier recours
#define NAV_SPIN_PWM 180         // PWM du spin

// --- Vitesse adaptative ---
#define NAV_CONFIDENCE_DIST_MM                                                 \
  1500 // Au-delà = pleine vitesse, en dessous = réduit

// --- Sentinel ---
#define NAV_NO_PATH -999

// ============================================================
//  API
// ============================================================

/**
 * @brief Calcule l'angle de cap optimal.
 *        0° = devant, positif = gauche, négatif = droite.
 *        Retourne NAV_NO_PATH si bloqué.
 */
float computeNaiveHeading();

/**
 * @brief Tâche FreeRTOS de navigation naïve.
 */
void naiveNavigationTask(void *pvParameters);

#endif
