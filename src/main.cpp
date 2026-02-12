#include "IMU.h"
#include "config.h"
#include "globals.h"
#include "lidar_manager.h"
#include "motor_control.h"
#include "naive_navigation.h"
#include "speed_estimator.h"

#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>

// ===== micro-ROS GLOBAL =====
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t scan_pub;
rcl_publisher_t obstacle_pub;
rcl_publisher_t imu_pub;

// ===== MESSAGES GLOBAUX =====
sensor_msgs__msg__LaserScan scan_msg;
std_msgs__msg__Bool obstacle_msg;
sensor_msgs__msg__Imu imu_msg;

// ===== ENCODEURS =====
#define ENCODER_LEFT_PIN 35
#define ENCODER_RIGHT_PIN 34
#define SSID "Pile AA"
#define PASS "3011906andy"
#define AGENT_IP "192.168.137.205"
#define AGENT_PORT 8888

// Flag pour savoir si micro-ROS est connecté
volatile bool microRosConnected = false;

void IRAM_ATTR encoderLeftISR() { onEncoderLeftPulse(); }
void IRAM_ATTR encoderRightISR() { onEncoderRightPulse(); }

// ===== TEST MOTEURS AU DÉMARRAGE =====
void testMotorsStartup() {
  // Petite pause pour laisser l'alimentation se stabiliser
  delay(500);

  Serial.println("[TEST] Motors: FORWARD 1s...");
  channelBCtrl(180); // Gauche avance (PWM modéré pour éviter chute de tension)
  channelACtrl(180); // Droite avance
  delay(1000);

  stopMotors();
  delay(500);

  Serial.println("[TEST] Motors: BACKWARD 1s...");
  channelBCtrl(-180); // Gauche recule
  channelACtrl(-180); // Droite recule
  delay(1000);

  stopMotors();
  delay(500);
  Serial.println("[TEST] Motors: DONE");
}

// ===== Tentative de connexion micro-ROS avec timeout =====
bool initMicroRos() {
  Serial.println("[uROS] Connecting to WiFi + agent...");
  set_microros_wifi_transports(SSID, PASS, AGENT_IP, AGENT_PORT);

  allocator = rcl_get_default_allocator();

  // Tenter l'init avec gestion d'erreur
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.printf("[uROS] FAIL: rclc_support_init returned %d\n", (int)ret);
    return false;
  }

  ret = rclc_node_init_default(&node, "esp32_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.printf("[uROS] FAIL: rclc_node_init returned %d\n", (int)ret);
    return false;
  }

  ret = rclc_publisher_init_default(
      &scan_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  if (ret != RCL_RET_OK) {
    Serial.printf("[uROS] FAIL: scan_pub init returned %d\n", (int)ret);
    return false;
  }

  ret = rclc_publisher_init_default(
      &obstacle_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/obstacle");
  if (ret != RCL_RET_OK) {
    Serial.printf("[uROS] FAIL: obstacle_pub init returned %d\n", (int)ret);
    return false;
  }

  ret = rclc_publisher_init_default(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu");
  if (ret != RCL_RET_OK) {
    Serial.printf("[uROS] FAIL: imu_pub init returned %d\n", (int)ret);
    return false;
  }

  Serial.println("[uROS] OK — all publishers ready");
  return true;
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  Serial.println("=== ESP32 WAVE ROVER START ===");

  // Afficher la RAM libre au démarrage
  Serial.printf("[MEM] Free heap: %u bytes\n", ESP.getFreeHeap());

  // --- Hardware init AVANT micro-ROS (moins de RAM utilisée) ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  initLidar();
  imuInit();
  initMotors();

  // --- Test moteurs ---
  testMotorsStartup();

  initSpeedEstimator();

  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightISR,
                  RISING);

  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();

  memset(&ctrlData, 0, sizeof(ctrlData));

  Serial.printf("[MEM] Free heap before uROS: %u bytes\n", ESP.getFreeHeap());

  // --- micro-ROS init avec gestion d'erreur ---
  microRosConnected = initMicroRos();
  if (!microRosConnected) {
    Serial.println("[uROS] WARNING: micro-ROS init failed!");
    Serial.println("[uROS] Robot will run without ROS publishing.");
    Serial.println(
        "[uROS] Check: 1) WiFi SSID/password 2) Agent IP 3) Agent running");
  }

  Serial.printf("[MEM] Free heap after uROS: %u bytes\n", ESP.getFreeHeap());

  // --- Tasks ---
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 5, NULL, 1);

  // Ne lancer la tâche micro-ROS que si connecté
  if (microRosConnected) {
    xTaskCreatePinnedToCore(microRosLidarTask, "uRosLidar", 16384, NULL, 4,
                            NULL, 1);
    xTaskCreatePinnedToCore(imuTask, "ImuTask", 8192, NULL, 3, NULL, 0);
  }

  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(naiveNavigationTask, "NavNaive", 4096, NULL, 2, NULL,
                          1);

  Serial.printf("[MEM] Free heap after tasks: %u bytes\n", ESP.getFreeHeap());
  Serial.println("=== SYSTEM READY ===");
}

void loop() {
  // Afficher la santé du système toutes les 10 secondes
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck > 10000) {
    lastHealthCheck = millis();
    Serial.printf(
        "[HEALTH] Heap: %u | uROS: %s | Obstacle: %s | LiDAR pts: %d\n",
        ESP.getFreeHeap(), microRosConnected ? "OK" : "DISCONNECTED",
        obstacleDetected ? "YES" : "no", pointsAvailable);
  }
  vTaskDelay(pdMS_TO_TICKS(500));
}
