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

void IRAM_ATTR encoderLeftISR() { onEncoderLeftPulse(); }
void IRAM_ATTR encoderRightISR() { onEncoderRightPulse(); }

// ===== TEST MOTEURS AU DÉMARRAGE =====
void testMotorsStartup() {
  Serial.println("[TEST] Motors: FORWARD 1s...");
  channelBCtrl(200); // Gauche avance
  channelACtrl(200); // Droite avance
  delay(1000);

  stopMotors();
  delay(300);

  Serial.println("[TEST] Motors: BACKWARD 1s...");
  channelBCtrl(-200); // Gauche recule
  channelACtrl(-200); // Droite recule
  delay(1000);

  stopMotors();
  delay(300);
  Serial.println("[TEST] Motors: DONE");
}

void setup() {

  delay(3000);
  Serial.begin(115200);
  Serial.println("=== ESP32 WAVE ROVER START ===");

  set_microros_wifi_transports(SSID, PASS, AGENT_IP, AGENT_PORT);

  // ===== micro-ROS INIT UNIQUE =====
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Publishers
  rclc_publisher_init_default(
      &scan_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");

  rclc_publisher_init_default(&obstacle_pub, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                              "/obstacle");

  rclc_publisher_init_default(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu");

  initLidar();
  imuInit();
  initMotors();
  testMotorsStartup(); // Test moteurs : avancer 1s, reculer 1s
  initSpeedEstimator();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

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

  // Tasks capteurs uniquement
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(microRosLidarTask, "microRosLidarTask", 8192, NULL, 5,
                          NULL, 1);
  xTaskCreatePinnedToCore(imuTask, "ImuTask", 8192, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(naiveNavigationTask, "NavNaive", 4096, NULL, 3, NULL,
                          1);

  Serial.println("=== SYSTEM READY ===");
}

void loop() { vTaskDelay(100); }
