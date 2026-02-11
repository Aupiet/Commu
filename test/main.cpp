#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include "lidar_manager.h"
#include "communication.h"
#include "display_manager.h"

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 WAVE ROVER v3.0 ===\n");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Init OLED
  initDisplay();
  
  // Init LiDAR
  initLidar();
  
  // Init Motors
  initMotors();
  testMotors();
  
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
  xTaskCreatePinnedToCore(communicationTask, "ESPNOW", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(streamingTask, "TCPStream", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(oledTask, "OLED", 4096, NULL, 1, NULL, 0);
  
  Serial.println("\n=== SYSTEM READY ===\n");
  
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