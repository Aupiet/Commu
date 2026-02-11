#ifndef NAIVE_NAVIGATION_H
#define NAIVE_NAVIGATION_H

#include <Arduino.h>

// ============================================================
//  SUPERVARIABLES — Modifier ici pour ajuster le comportement
// ============================================================

// --- Robot (Wave Rover) ---
#define NAV_ROBOT_WIDTH_MM 168.0f  // Largeur du Wave Rover (mm)
#define NAV_SAFETY_MARGIN_MM 50.0f // Marge latérale de sécurité (mm)
#define NAV_MIN_PASSAGE_MM                                                     \
  (NAV_ROBOT_WIDTH_MM + 2.0f * NAV_SAFETY_MARGIN_MM) // 268 mm

// --- LiDAR (LD06) ---
#define NAV_MAX_RANGE_MM 6000 // Portée max considérée (mm)
#define NAV_MIN_RANGE_MM 50   // En dessous = bruit LD06
#define NAV_SCAN_RESOLUTION 1 // Résolution angulaire (degrés)

// --- Comportement ---
#define NAV_FORWARD_HALF_ANGLE 45.0f // Demi-cône de recherche avant (±45°)
#define NAV_FORWARD_SPEED 150        // PWM d'avance (0-255)
#define NAV_TURN_SPEED 120           // PWM de rotation sur place
#define NAV_ANGLE_GAIN 3.0f    // Gain proportionnel angle → correction moteur
#define NAV_TASK_PERIOD_MS 100 // Période de la boucle (10 Hz)

// --- Sentinel ---
#define NAV_NO_PATH -999 // Aucun chemin viable trouvé

// ============================================================
//  API
// ============================================================

/**
 * @brief Calcule l'angle de cap optimal à partir du buffer LiDAR.
 * @return Angle en degrés (0 = devant, positif = gauche, négatif = droite).
 *         Retourne NAV_NO_PATH si aucun passage n'est viable.
 */
float computeNaiveHeading();

/**
 * @brief Tâche FreeRTOS de navigation naïve.
 *        Vérifie currentNavMode == NAV_NAIVE, calcule le cap, écrit motorCmd.
 */
void naiveNavigationTask(void *pvParameters);

#endif
