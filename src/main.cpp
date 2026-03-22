#include "IMU.h"
#include "config.h"
#include "globals.h"
#include "lidar_manager.h"
#include "motor_control.h"
#include "naive_navigation.h"


#include <Arduino.h>
#include <micro_ros_arduino.h>

// ===== CONFIG =====
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

  // Mode navigation naïve activé
  currentNavMode = NAV_NAIVE;

  Serial.printf("[MEM] Free heap before tasks: %u bytes\n", ESP.getFreeHeap());

  // ===== TÂCHES =====
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosLidarTask, "uRosLidar", 24000, NULL, 5, NULL,
                          1);
  xTaskCreatePinnedToCore(imuTask, "ImuTask", 8192, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(naiveNavigationTask, "NavNaive", 4096, NULL, 2, NULL,
                          1);

  Serial.printf("[MEM] Free heap after tasks: %u bytes\n", ESP.getFreeHeap());
  Serial.println("=== SYSTEM READY ===");
}

void loop() {
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck > 10000) {
    lastHealthCheck = millis();
    Serial.printf(
        "[HEALTH] Heap: %u | Obstacle: %s | MinDist: %d | LiDAR pts: %d\n",
        ESP.getFreeHeap(), obstacleDetected ? "YES" : "no", minObstacleDistance,
        pointsAvailable);
  }
  vTaskDelay(pdMS_TO_TICKS(500));
}