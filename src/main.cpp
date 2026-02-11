#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>
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

void IRAM_ATTR encoderLeftISR() {
  onEncoderLeftPulse();
}
void IRAM_ATTR encoderRightISR() {
  onEncoderRightPulse();
}

void setup() {
  delay(5000);
  Serial.begin(115200);
  Serial.println("\n=== ESP32 WAVE ROVER v3.2 (Lidar Optimized) ===\n");

  // Transport micro-ROS (USB ou WiFi)
  set_microros_transports();
  Serial.println("MICRO-ROS START");

  initLidar();

  Serial.println("System ready (LiDAR + micro-ROS)");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // I2C Fixe pour ESP32
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  initDisplay();
  initMotors();
  imuInit(); 
  initSpeedEstimator();
  
  // Encodeurs
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR, RISING);
  
  initCommunication();

  // Mutex
  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();
  
  // NOTE: packetQueue a été supprimé car on traite le LiDAR en direct maintenant
  
  memset(&ctrlData, 0, sizeof(ctrlData));
  
  // --- LANCEMENT DES TÂCHES (Correction de ton erreur ici) ---
  
  // Priorité 5 (Max) : Nouvelle tâche unique LiDAR optimisée
  xTaskCreatePinnedToCore(lidarTask, "LidarFast", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosLidarTask, "microROS_Lidar", 24000, NULL, 5, NULL, 1);

  
  // Priorité 3 : Moteurs
  //xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  
  // Autres tâches
  xTaskCreatePinnedToCore(imuTask, "microROS_IMU", 8192, NULL, 4, NULL, 0);

  //xTaskCreatePinnedToCore(speedEstimatorTask, "SpeedEst", 4096, NULL, 4, NULL, 1);
  //xTaskCreatePinnedToCore(communicationTask, "ESPNOW", 4096, NULL, 2, NULL, 0);
  //xTaskCreatePinnedToCore(streamingTask, "TCPStream", 8192, NULL, 1, NULL, 0);

  Serial.println("\n=== SYSTEM READY ===\n");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Statut toutes les 2s
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 2000) {
     if(obstacleDetected) Serial.println("!!! OBSTACLE !!!");
     lastStats = millis();
  }
  vTaskDelay(100);
}