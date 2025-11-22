#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include "lidar_manager.h"
#include "communication.h"
#include "display_manager.h"
#include "speed_estimator.h"
#include "IMU.h"
#include <Wire.h>

// CONFIGURATION PINS ENCODEURS
#define ENCODER_LEFT_PIN 35
#define ENCODER_RIGHT_PIN 34

// ISR Callbacks encodeurs
void IRAM_ATTR encoderLeftISR() {
  onEncoderLeftPulse();
}

void IRAM_ATTR encoderRightISR() {
  onEncoderRightPulse();
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 WAVE ROVER v3.1 (Fixed I2C) ===\n");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // --- CORRECTION CRITIQUE I2C ---
  // On force les pins 32 et 33. 
  // Si on ne le fait pas, Wire prend 21/22 et coupe les moteurs.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 
  // -------------------------------

  initDisplay(); // N'appelle plus Wire.begin()
  initLidar();
  initMotors();  // Pins 21 et 22 maintenant libres
  
  // Init IMU & Speed
  imuInit(); 
  initSpeedEstimator();
  
  // Encodeurs
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR, RISING);
  
  initCommunication();
  
  // Synchro
  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();
  packetQueue = xQueueCreate(50, PACKET_SIZE);
  
  memset(&ctrlData, 0, sizeof(ctrlData));
  memset(pointBuffer, 0, sizeof(pointBuffer));
  
  // Tasks
  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(lidarReadTask, "LidarRead", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(lidarProcessTask, "LidarProcess", 8192, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(speedEstimatorTask, "SpeedEst", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(communicationTask, "ESPNOW", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(streamingTask, "TCPStream", 8192, NULL, 1, NULL, 0);

  Serial.println("\n=== SYSTEM READY ===\n");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 10000) {
    Serial.printf("Stats - Cmd:%u Speed:%.2fm/s (Enc:%.2f IMU:%.2f) Obs:%s\n", 
                  receivedCommands, 
                  estimatedSpeed.speed_m_per_sec, 
                  estimatedSpeed.encoder_speed_mm_per_sec/1000.0,
                  estimatedSpeed.imu_speed_mm_per_sec/1000.0,
                  obstacleDetected ? "YES" : "NO");
    lastStats = millis();
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}