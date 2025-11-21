#include "speed_estimator.h"
#include "globals.h"
#include <math.h>
#include "IMU.h"

// ========== ÉTATS INTERNES ==========
static volatile uint32_t pulseCountLeft = 0;
static volatile uint32_t pulseCountRight = 0;
static volatile unsigned long lastPulseTimeLeft = 0;
static volatile unsigned long lastPulseTimeRight = 0;

static SpeedEstimate currentEstimate = {
    .speed_mm_per_sec = 0.0f,
    .speed_m_per_sec = 0.0f,
    .encoder_speed_mm_per_sec = 0.0f,
    .imu_speed_mm_per_sec = 0.0f,
    .confidence = 0.0f,
    .last_update_ms = 0
};

static SemaphoreHandle_t speedMutex = nullptr;
static float imuIntegratedSpeed = 0.0f;  // Intégration accélération IMU
static unsigned long lastEstimatorUpdateMs = 0;

// ========== INITIALISATION ==========
void initSpeedEstimator() {
    speedMutex = xSemaphoreCreateMutex();
    pulseCountLeft = 0;
    pulseCountRight = 0;
    lastPulseTimeLeft = millis();
    lastPulseTimeRight = millis();
    imuIntegratedSpeed = 0.0f;
    lastEstimatorUpdateMs = millis();
    Serial.println("[SPEED] Estimateur vitesse initialise");
}

// ========== CALLBACKS ENCODEURS ==========
void onEncoderLeftPulse() {
    pulseCountLeft++;
    lastPulseTimeLeft = micros();
}

void onEncoderRightPulse() {
    pulseCountRight++;
    lastPulseTimeRight = micros();
}

// ========== TÂCHE ESTIMATEUR (FreeRTOS) ==========
void speedEstimatorTask(void *pvParameters) {
    Serial.printf("SpeedEstimator task running on core %d\n", xPortGetCoreID());
    
    float filteredSpeed = 0.0f;
    unsigned long lastCalcMs = millis();
    
    while (true) {
        unsigned long nowMs = millis();
        float dtSec = (float)(nowMs - lastCalcMs) * 0.001f;  // dt en secondes
        if (dtSec < 0.01f) dtSec = 0.01f;  // Minimum 10ms
        
        // --- 1. Récupérer données encodeurs ---
        uint32_t pulseL = 0, pulseR = 0;
        if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            pulseL = pulseCountLeft;
            pulseR = pulseCountRight;
            pulseCountLeft = 0;
            pulseCountRight = 0;
            xSemaphoreGive(speedMutex);
        }
        
        // Calcul vitesse encodeur (moyenne L+R)
        float totalPulses = (float)(pulseL + pulseR) * 0.5f;
        float distanceMm = totalPulses * ENCODER_DIST_PER_PULSE;
        float encoderSpeedMmSec = distanceMm / dtSec;
        
        // --- 2. Récupérer données IMU (accélération) ---
        float imuSpeedMmSec = 0.0f;
        float accelMagnitude = 0.0f;
        
        if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Récupérer accél brute depuis globals
            float ax = imuAccelRaw.X;
            float ay = imuAccelRaw.Y;
            
            // Magnitude accélération horizontale (ignorer Z qui est gravité)
            accelMagnitude = sqrtf(ax*ax + ay*ay);
            
            // Intégration accélération -> vitesse (attention : dérive lente)
            if (accelMagnitude > MIN_ACCEL_MAGNITUDE) {
                imuIntegratedSpeed += (accelMagnitude * 0.001f) * dtSec;  // mg -> m/s²
                imuIntegratedSpeed = constrain(imuIntegratedSpeed, 0.0f, MAX_EXPECTED_SPEED);
            } else {
                imuIntegratedSpeed *= 0.95f;  // Décroissance lente si pas de mouvement
            }
            
            imuSpeedMmSec = imuIntegratedSpeed * 1000.0f;  // m/s -> mm/s
            xSemaphoreGive(speedMutex);
        }
        
        // --- 3. Fusion Kalman simplifié ---
        float fusedSpeedMmSec = 0.0f;
        float confidence = 0.5f;
        
        if (encoderSpeedMmSec > 1.0f || imuSpeedMmSec > 1.0f) {
            // Pondération adaptative selon fiabilité encodeur
            float encoderReliability = (encoderSpeedMmSec > 0.1f) ? 1.0f : 0.5f;
            float imuReliability = (accelMagnitude > MIN_ACCEL_MAGNITUDE) ? 0.8f : 0.3f;
            
            float totalWeight = (ENCODER_WEIGHT * encoderReliability) + (IMU_ACCEL_WEIGHT * imuReliability);
            
            if (totalWeight > 0.01f) {
                fusedSpeedMmSec = 
                    (encoderSpeedMmSec * ENCODER_WEIGHT * encoderReliability +
                     imuSpeedMmSec * IMU_ACCEL_WEIGHT * imuReliability) / totalWeight;
                
                confidence = totalWeight / (ENCODER_WEIGHT + IMU_ACCEL_WEIGHT);
            }
        }
        
        // Saturation et filtre IIR
        fusedSpeedMmSec = constrain(fusedSpeedMmSec, 0.0f, MAX_EXPECTED_SPEED * 1000.0f);
        filteredSpeed = filteredSpeed + SPEED_FILTER_ALPHA * (fusedSpeedMmSec - filteredSpeed);
        
        // --- 4. Mettre à jour structure globale ---
        if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            currentEstimate.speed_mm_per_sec = filteredSpeed;
            currentEstimate.speed_m_per_sec = filteredSpeed * 0.001f;
            currentEstimate.encoder_speed_mm_per_sec = encoderSpeedMmSec;
            currentEstimate.imu_speed_mm_per_sec = imuSpeedMmSec;
            currentEstimate.confidence = confidence;
            currentEstimate.last_update_ms = nowMs;
            xSemaphoreGive(speedMutex);
        }
        
        lastCalcMs = nowMs;
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 20 Hz
    }
}

// ========== GETTER ==========
SpeedEstimate getSpeedEstimate() {
    SpeedEstimate result = {0};
    
    if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        result = currentEstimate;
        xSemaphoreGive(speedMutex);
    }
    
    return result;
}

// ========== RESET ==========
void resetSpeedEstimator() {
    if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        pulseCountLeft = 0;
        pulseCountRight = 0;
        imuIntegratedSpeed = 0.0f;
        currentEstimate.speed_mm_per_sec = 0.0f;
        currentEstimate.speed_m_per_sec = 0.0f;
        xSemaphoreGive(speedMutex);
    }
}