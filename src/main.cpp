#include "IMU.h"
#include "config.h"
#include "globals.h"
#include "lidar_manager.h"
#include "motor_control.h"


#include <Arduino.h>
#include <micro_ros_arduino.h>

// ===== CONFIG =====
#define SSID "Pile AA"
#define PASS "3011906andy"
#define AGENT_IP "192.168.137.205"
#define AGENT_PORT 8888

#define FORWARD_PWM 150

// ===== TÂCHE SIMPLE : avancer + stop mur =====
void simpleForwardTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println("[NAV] Simple forward task started");

  bool wasStopped = false;

  while (true) {
    if (obstacleDetected) {
      stopMotors();
      if (!wasStopped) {
        Serial.printf("[NAV] STOP! Obstacle at %d mm\n", minObstacleDistance);
        wasStopped = true;
      }
    } else {
      channelBCtrl(FORWARD_PWM);
      channelACtrl(FORWARD_PWM);
      if (wasStopped) {
        Serial.println("[NAV] GO - obstacle cleared");
        wasStopped = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
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

  // Mutexes
  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();

  memset(&ctrlData, 0, sizeof(ctrlData));

  Serial.printf("[MEM] Free heap before tasks: %u bytes\n", ESP.getFreeHeap());

  // ===== TÂCHES =====
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosLidarTask, "uRosLidar", 24000, NULL, 5, NULL,
                          1);
  xTaskCreatePinnedToCore(imuTask, "ImuTask", 8192, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(simpleForwardTask, "Forward", 4096, NULL, 2, NULL, 1);

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
