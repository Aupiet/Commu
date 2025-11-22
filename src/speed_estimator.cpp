#include "speed_estimator.h"
#include "globals.h"
#include "IMU.h"
#include "motor_control.h"
#include <math.h> // Nécessaire pour sin() et PI

// Note: estimatedSpeed est définie dans globals.cpp

volatile unsigned long leftPulseCount = 0;
volatile unsigned long rightPulseCount = 0;

unsigned long lastCalcTime = 0;
unsigned long lastLeftPulses = 0;
unsigned long lastRightPulses = 0;
float velocityEstimate = 0.0f; 

SemaphoreHandle_t encoderMutex;

void IRAM_ATTR onEncoderLeftPulse() {
  leftPulseCount++;
}

void IRAM_ATTR onEncoderRightPulse() {
  rightPulseCount++;
}

void initSpeedEstimator() {
  encoderMutex = xSemaphoreCreateMutex();
  resetSpeedEstimator();
  Serial.println("Speed Estimator Initialized");
}

void resetSpeedEstimator() {
  // Reset variables...
  estimatedSpeed.speed_mm_per_sec = 0;
  estimatedSpeed.speed_m_per_sec = 0;
  estimatedSpeed.encoder_speed_mm_per_sec = 0;
  estimatedSpeed.imu_speed_mm_per_sec = 0;
  estimatedSpeed.confidence = 0;
  estimatedSpeed.last_update_ms = 0;
  
  velocityEstimate = 0;
  
  if (encoderMutex) {
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    leftPulseCount = 0;
    rightPulseCount = 0;
    lastLeftPulses = 0;
    lastRightPulses = 0;
    xSemaphoreGive(encoderMutex);
  }
  lastCalcTime = millis();
}

void speedEstimatorTask(void *pvParameters) {
  const int loop_delay_ms = 20; // 50Hz
  
  while (true) {
    unsigned long now = millis();
    float dt = (now - lastCalcTime) / 1000.0f;
    
    if (dt >= 0.02f) {
      // 1. Mise à jour IMU (lit accéléro + calcule angles)
      updateIMUData(); 
      
      // 2. Lecture Encodeurs
      unsigned long currentLeft = 0;
      unsigned long currentRight = 0;
      
      if (xSemaphoreTake(encoderMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        currentLeft = leftPulseCount;
        currentRight = rightPulseCount;
        xSemaphoreGive(encoderMutex);
      }
      
      long deltaLeft = currentLeft - lastLeftPulses;
      long deltaRight = currentRight - lastRightPulses;
      
      lastLeftPulses = currentLeft;
      lastRightPulses = currentRight;
      lastCalcTime = now;
      
      // 3. Calcul vitesse Encodeurs (référence stable)
      float distLeft = deltaLeft * ENCODER_DIST_PER_PULSE;
      float distRight = deltaRight * ENCODER_DIST_PER_PULSE;
      float distAvg = (distLeft + distRight) / 2.0f;
      
      float speedEnc = 0;
      if (dt > 0) speedEnc = distAvg / dt; 
      
      // Sens de rotation (si pas de pin direction sur encodeurs, on utilise le PWM comme indice)
      if (currentSpeedPWM < 0) speedEnc = -fabs(speedEnc);
      
      // --- CORRECTION DU CALCUL IMU ---
      
      // Conversion de l'accélération brute (supposée en 'g') en 'mm/s²'
      // ATTENTION : stAccelRawData contient la gravité !
      // Il faut compenser l'inclinaison (Pitch) du robot.
      
      // Conversion degrés -> radians
      float pitchRad = stAngles.pitch * (PI / 180.0f); 
      
      // Calcul de la composante de gravité sur l'axe X
      // Si le robot monte (pitch positif), la gravité tire vers l'arrière (-X)
      // La formule dépend de l'orientation de votre puce, souvent : g_x = sin(pitch)
      float gravityCompG = sin(pitchRad); 
      
      // Accélération Linéaire = Accélération Mesurée - Gravité
      float linearAccelG = stAccelRawData.X - gravityCompG; 
      
      // Conversion en mm/s² (1g ~= 9806.65 mm/s²)
      float accelFwd = linearAccelG * 9806.65f; 

      // Deadband (Zone morte) : Filtrer le bruit électronique
      if (fabs(linearAccelG) < 0.05f) { // Ignorer si inférieur à ~0.05g
        accelFwd = 0;
      }
      
      // Intégration : Vitesse = Vitesse_Precedente + Acceleration * temps
      float velPrediction = velocityEstimate + (accelFwd * dt);
      
      // --- FUSION (Filtre Complémentaire) ---
      
      // Si on est censé être à l'arrêt (PWM 0 et encodeurs immobiles)
      if (currentSpeedPWM == 0 && fabs(speedEnc) < 10) {
        // On force la vitesse vers 0 rapidement pour éviter la dérive
        velocityEstimate = velocityEstimate * 0.5f; 
        if (fabs(velocityEstimate) < 5.0f) velocityEstimate = 0;
        velPrediction = 0; // Reset de l'intégrale IMU
      } else {
        // Fusion : On fait confiance aux encodeurs à long terme (90-95%) 
        // et à l'IMU pour les changements rapides (5-10%)
        // ENCODER_WEIGHT devrait être autour de 0.90 ou 0.95
        velocityEstimate = (velPrediction * (1.0f - ENCODER_WEIGHT)) + (speedEnc * ENCODER_WEIGHT);
      }

      // Mise à jour structure globale
      estimatedSpeed.speed_mm_per_sec = velocityEstimate;
      estimatedSpeed.speed_m_per_sec = velocityEstimate / 1000.0f;
      estimatedSpeed.encoder_speed_mm_per_sec = speedEnc;
      estimatedSpeed.imu_speed_mm_per_sec = velPrediction; 
      estimatedSpeed.last_update_ms = now;
      
      // Debug
      // Serial.print("Enc:"); Serial.print(speedEnc);
      // Serial.print(" IMU_Raw:"); Serial.print(accelFwd);
      // Serial.print(" Est:"); Serial.println(velocityEstimate);
    }
    vTaskDelay(loop_delay_ms / portTICK_PERIOD_MS);
  }
}