#include "config.h"
#include "globals.h"
#include "lidar_manager.h"
#include "motor_control.h"
// #include "naive_navigation.h"   // COMMENTÉ pour debug LIDAR
// #include "IMU.h"                // COMMENTÉ pour debug LIDAR
// #include "speed_estimator.h"    // COMMENTÉ pour debug LIDAR

#include <Arduino.h>
#include <micro_ros_arduino.h>

// ===== ENCODEURS =====
#define ENCODER_LEFT_PIN 35
#define ENCODER_RIGHT_PIN 34
#define SSID "Pile AA"
#define PASS "3011906andy"
#define AGENT_IP "192.168.137.205"
#define AGENT_PORT 8888

// ===== TEST MOTEURS AU DÉMARRAGE =====
void testMotorsStartup() {
  delay(500);

  Serial.println("[TEST] Motors: FORWARD 1s...");
  channelBCtrl(180);
  channelACtrl(180);
  delay(1000);

  stopMotors();
  delay(500);

  Serial.println("[TEST] Motors: BACKWARD 1s...");
  channelBCtrl(-180);
  channelACtrl(-180);
  delay(1000);

  stopMotors();
  delay(500);
  Serial.println("[TEST] Motors: DONE");
}

void setup() {

  delay(2000);
  Serial.begin(115200);
  Serial.println("=== ESP32 WAVE ROVER START ===");
  Serial.printf("[MEM] Free heap: %u bytes\n", ESP.getFreeHeap());

  // WiFi transport pour micro-ROS
  set_microros_wifi_transports(SSID, PASS, AGENT_IP, AGENT_PORT);
  Serial.println("[WiFi] Transport configured");
  Serial.printf("[MEM] Free heap after WiFi: %u bytes\n", ESP.getFreeHeap());

  // Init hardware
  initLidar();
  initMotors();

  // Test moteurs
  testMotorsStartup();

  // Mutexes
  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();

  memset(&ctrlData, 0, sizeof(ctrlData));

  Serial.printf("[MEM] Free heap before tasks: %u bytes\n", ESP.getFreeHeap());

  // ===== TÂCHES LIDAR UNIQUEMENT =====
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosLidarTask, "uRosLidar", 24000, NULL, 5, NULL,
                          1);

  // ===== COMMENTÉ POUR DEBUG LIDAR =====
  // xTaskCreatePinnedToCore(imuTask, "ImuTask", 8192, NULL, 4, NULL, 0);
  // xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL,
  // 1); xTaskCreatePinnedToCore(naiveNavigationTask, "NavNaive", 4096, NULL, 3,
  // NULL, 1);

  Serial.printf("[MEM] Free heap after tasks: %u bytes\n", ESP.getFreeHeap());
  Serial.println("=== SYSTEM READY (LIDAR ONLY MODE) ===");
}

void loop() {
  // Health check toutes les 10 secondes
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck > 10000) {
    lastHealthCheck = millis();
    Serial.printf("[HEALTH] Heap: %u | Obstacle: %s | LiDAR pts: %d\n",
                  ESP.getFreeHeap(), obstacleDetected ? "YES" : "no",
                  pointsAvailable);
  }
  vTaskDelay(pdMS_TO_TICKS(500));
}
