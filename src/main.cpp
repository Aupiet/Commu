#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include "lidar_manager.h"
#include "communication.h"
#include "display_manager.h"
// À AJOUTER en haut du fichier :
#include "speed_estimator.h"

// CONFIGURATION PINS ENCODEURS (à adapter selon votre setup)
#define ENCODER_LEFT_PIN 35   // GPIO35 - Encodeur gauche (ADC compatible ou GPIO)
#define ENCODER_RIGHT_PIN 34  // GPIO34 - Encodeur droit (ADC compatible ou GPIO)

// ISR Callbacks encodeurs
void IRAM_ATTR encoderLeftISR() {
  onEncoderLeftPulse();
}

void IRAM_ATTR encoderRightISR() {
  onEncoderRightPulse();
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 WAVE ROVER v3.1 (with Speed Fusion) ===\n");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Init OLED
  initDisplay();
  
  // Init LiDAR
  initLidar();
  
  // Init Motors
  initMotors();
  
  // --- NOUVEAU : Init Speed Estimator ---
  initSpeedEstimator();
  
  // --- NOUVEAU : Setup encodeurs avec interruptions ---
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR, RISING);
  Serial.println("Encoders initialized with ISR");
  
  // Init WiFi & ESP-NOW
  initCommunication();
  
  // Init synchronization
  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();
  packetQueue = xQueueCreate(50, PACKET_SIZE);
  
  memset(&ctrlData, 0, sizeof(ctrlData));
  memset(pointBuffer, 0, sizeof(pointBuffer));
  
  // Create tasks
  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(lidarReadTask, "LidarRead", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(lidarProcessTask, "LidarProcess", 8192, NULL, 4, NULL, 1);
  
  // --- NOUVEAU : Tâche estimateur vitesse (Core 0) ---
  xTaskCreatePinnedToCore(speedEstimatorTask, "SpeedEst", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(communicationTask, "ESPNOW", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(streamingTask, "TCPStream", 8192, NULL, 1, NULL, 0);

  Serial.println("\n=== SYSTEM READY (Speed Fusion Active) ===\n");
  
  // Blink LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 10000) {
    Serial.printf("Stats - Cmd:%u FB:%u Pkt:%lu Pts:%lu Obs:%s\n", 
                  receivedCommands, sentFeedback, packetsReceived, 
                  totalPointsProcessed, obstacleDetected ? "YES" : "NO");
    lastStats = millis();
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}