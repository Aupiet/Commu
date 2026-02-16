#include "lidar_manager.h"
#include "config.h"
#include "globals.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>

HardwareSerial lidarSerial(1);

// ===== micro-ROS =====
static rcl_publisher_t scan_pub;
static sensor_msgs__msg__LaserScan scan_msg;

// Subscriber /naif
static rcl_subscription_t naif_sub;
static std_msgs__msg__Bool naif_msg;

// Partagés avec imuTask
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_node_t uros_node;
rclc_support_t uros_support;
rcl_allocator_t uros_allocator;
volatile bool microRosReady = false;

// Callback /naif : 1 = démarrer naïf, 0 = arrêter
static void naifCallback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    naifEnabled = true;
    Serial.println("[NAIF] Enabled (received true)");
  } else {
    naifEnabled = false;
    Serial.println("[NAIF] Disabled (received false)");
  }
}

// ===== PARAMÈTRES =====
#define DIST_STOP_MM 350
#define OBSTACLE_HOLD_MS 400

// =====================

void initLidar() {
  lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("LiDAR initialized");
}

static inline void polarToCartesian(float a_deg, uint16_t d_mm, float &x,
                                    float &y) {
  float a = a_deg * 0.0174532925f;
  float m = d_mm * 0.001f;
  x = sinf(a) * m;
  y = cosf(a) * m;
}

// ===== TÂCHE LiDAR TEMPS RÉEL =====
void lidarTask(void *pv) {
  uint8_t buf[PACKET_SIZE];
  unsigned long lastObstacleTime = 0;

  while (true) {
    if (lidarSerial.available() && lidarSerial.read() == 0x54) {

      if (lidarSerial.readBytes(buf, PACKET_SIZE - 1) != PACKET_SIZE - 1)
        continue;
      if (buf[0] != 0x2C)
        continue;

      float start = (buf[3] | (buf[4] << 8)) / 100.0f;
      float end = (buf[41] | (buf[42] << 8)) / 100.0f;
      if (end < start)
        end += 360.0f;

      float step = (end - start) / (MEAS_PER_PACKET - 1);

      bool obstacleLocal = false;
      uint16_t minDist = 9999;

      if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(2)) == pdTRUE) {

        for (int i = 0; i < MEAS_PER_PACKET; i++) {
          int b = 5 + i * 3;
          uint16_t dist = buf[b] | (buf[b + 1] << 8);
          uint8_t conf = buf[b + 2];

          if (conf < 100 || dist == 0 || dist > 12000)
            continue;

          float ang = start + step * i;
          if (ang >= 360)
            ang -= 360;

          LidarPoint p;
          p.angle = ang;
          p.distance = dist;
          p.confidence = conf;
          polarToCartesian(ang, dist, p.x, p.y);
          p.ts = millis();

          pointBuffer[pointWriteIndex] = p;
          pointWriteIndex = (pointWriteIndex + 1) % POINT_BUFFER_SIZE;
          if (pointsAvailable < POINT_BUFFER_SIZE)
            pointsAvailable++;

          bool inFront = (ang >= FRONT_ANGLE_MIN || ang <= FRONT_ANGLE_MAX);
          if (inFront && dist < DIST_STOP_MM) {
            obstacleLocal = true;
            minDist = min(minDist, dist);
          }
        }
        xSemaphoreGive(bufferMutex);
      }

      if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        if (obstacleLocal) {
          obstacleDetected = true;
          minObstacleDistance = minDist;
          lastObstacleTime = millis();
        } else if (obstacleDetected &&
                   millis() - lastObstacleTime > OBSTACLE_HOLD_MS) {
          obstacleDetected = false;
          minObstacleDistance = 9999;
        }
        lastLidarUpdate = millis();
        xSemaphoreGive(lidarMutex);
      }
    }
    vTaskDelay(1);
  }
}

// ===== TÂCHE micro-ROS =====
void microRosLidarTask(void *pv) {

  Serial.println("[uROS] Task started, waiting 2s...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  rclc_executor_t executor;
  uros_allocator = rcl_get_default_allocator();

  // Retry loop
  Serial.println("[uROS] Calling rclc_support_init...");
  rcl_ret_t ret;
  int attempts = 0;
  do {
    ret = rclc_support_init(&uros_support, 0, NULL, &uros_allocator);
    if (ret != RCL_RET_OK) {
      attempts++;
      Serial.printf("[uROS] support_init FAILED (ret=%d), attempt %d\n",
                    (int)ret, attempts);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  } while (ret != RCL_RET_OK && attempts < 30);

  if (ret != RCL_RET_OK) {
    Serial.println("[uROS] ABANDON: agent unreachable");
    vTaskDelete(NULL);
    return;
  }
  Serial.println("[uROS] support_init OK");

  rclc_node_init_default(&uros_node, "esp32_node", "", &uros_support);
  Serial.println("[uROS] Node OK");

  rclc_executor_init(&executor, &uros_support.context, 1, &uros_allocator);

  // Scan publisher (default = reliable, comme dans le code qui marchait)
  rclc_publisher_init_default(
      &scan_pub, &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  Serial.println("[uROS] scan_pub OK");

  // IMU publisher (best effort pour le message gros)
  rclc_publisher_init_best_effort(
      &imu_pub, &uros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu");
  Serial.println("[uROS] imu_pub OK");

  // Subscriber /naif (Bool)
  rclc_subscription_init_default(
      &naif_sub, &uros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/naif");
  rclc_executor_add_subscription(&executor, &naif_sub, &naif_msg, &naifCallback,
                                 ON_NEW_DATA);
  Serial.println("[uROS] /naif subscriber OK");

  microRosReady = true;
  Serial.println("[uROS] microRosReady = true");

  // Setup LaserScan
  scan_msg.header.frame_id.data = (char *)"laser";
  scan_msg.header.frame_id.size = strlen("laser");
  scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

  scan_msg.ranges.capacity = 360;
  scan_msg.ranges.size = 360;
  scan_msg.ranges.data = (float *)malloc(360 * sizeof(float));

  scan_msg.angle_min = 0.0f;
  scan_msg.angle_max = 2 * M_PI;
  scan_msg.angle_increment = (2 * M_PI) / 360.0f;
  scan_msg.range_min = 0.05f;
  scan_msg.range_max = 12.0f;

  Serial.println("[uROS] Entering publish loop");

  while (true) {
    // --- SCAN ---
    for (int i = 0; i < 360; i++)
      scan_msg.ranges.data[i] = INFINITY;

    if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      for (int i = 0; i < pointsAvailable; i++) {
        int idx = (pointWriteIndex - i + POINT_BUFFER_SIZE) % POINT_BUFFER_SIZE;
        int a = (int)pointBuffer[idx].angle;
        if (a >= 0 && a < 360)
          scan_msg.ranges.data[a] = pointBuffer[idx].distance * 0.001f;
      }
      xSemaphoreGive(bufferMutex);
    }

    scan_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    scan_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;

    rcl_publish(&scan_pub, &scan_msg, NULL);

    // --- IMU (données remplies par imuTask sur core 0) ---
    imu_msg.header.stamp.sec = scan_msg.header.stamp.sec;
    imu_msg.header.stamp.nanosec = scan_msg.header.stamp.nanosec;
    rcl_publish(&imu_pub, &imu_msg, NULL);

    // Traiter les subscriptions (callback /naif)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}
