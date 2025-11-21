#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <Arduino.h>

// Configuration paramètres fusion vitesse
#define WHEEL_RADIUS_MM 50.0f           // Rayon roue en mm
#define WHEEL_CIRCUMFERENCE_MM (2.0f * 3.14159265f * WHEEL_RADIUS_MM)
#define ENCODER_PPR 20                  // Pulses per revolution
#define ENCODER_DIST_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR) // mm par pulse

// Paramètres fusion (Kalman simplifié)
#define IMU_ACCEL_WEIGHT 0.3f           // Poids accélération IMU
#define ENCODER_WEIGHT 0.7f             // Poids encodeur
#define SPEED_FILTER_ALPHA 0.15f        // Filtre IIR (0.0-1.0)
#define MAX_EXPECTED_SPEED 5000.0f      // Vitesse max m/s (contrôle cohérence)
#define MIN_ACCEL_MAGNITUDE 50.0f       // mg, seuil détection mouvement

// Structure données vitesse estimée
typedef struct {
    float speed_mm_per_sec;             // Vitesse fusionnée (mm/s)
    float speed_m_per_sec;              // Vitesse (m/s)
    float encoder_speed_mm_per_sec;     // Vitesse encodeur brut
    float imu_speed_mm_per_sec;         // Vitesse IMU intégrée
    float confidence;                   // 0.0-1.0 (confiance estimée)
    unsigned long last_update_ms;       // Timestamp dernière mise à jour
} SpeedEstimate;

// Initialisation
void initSpeedEstimator();

// Callback pour pulses encodeurs (appeler depuis interrupt/poll)
// Gauche (moteur A)
void onEncoderLeftPulse();
// Droite (moteur B)
void onEncoderRightPulse();

// Tâche FreeRTOS de calcul vitesse fusionnée (Core 0 ou 1)
void speedEstimatorTask(void *pvParameters);

// Getter valeurs actuelles
SpeedEstimate getSpeedEstimate();

// Reset (si besoin redémarrage estimation)
void resetSpeedEstimator();

#endif