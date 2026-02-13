#include "lidar_manager.h"
#include "config.h"
#include "globals.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>

HardwareSerial lidarSerial(1);

// ===== micro-ROS =====
static rcl_publisher_t scan_pub;
static sensor_msgs__msg__LaserScan scan_msg;

// Partagés avec imuTask (non-static)
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_node_t uros_node;
rclc_support_t uros_support;
rcl_allocator_t uros_allocator;
volatile bool microRosReady = false;

// ===== PARAMÈTRES OBSTACLE =====
#define DIST_STOP_MM 400
#define FRONT_CONE_HALF 40 // ±40° devant le robot

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

          // Détection obstacle : cône frontal ±FRONT_CONE_HALF°
          // L'angle 0° = devant le robot pour le LD06 (vérifier selon montage)
          bool inFront =
              (ang <= FRONT_CONE_HALF || ang >= (360 - FRONT_CONE_HALF));
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
        } else {
          // Relâcher dès que le paquet frontal ne voit plus d'obstacle
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

  // Retry loop pour rclc_support_init
  Serial.println("[uROS] Calling rclc_support_init...");
  rcl_ret_t ret;
  int attempts = 0;
  do {
    ret = rclc_support_init(&uros_support, 0, NULL, &uros_allocator);
    if (ret != RCL_RET_OK) {
      attempts++;
      Serial.printf(
          "[uROS] support_init FAILED (ret=%d), attempt %d, retrying...\n",
          (int)ret, attempts);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  } while (ret != RCL_RET_OK && attempts < 30);

  if (ret != RCL_RET_OK) {
    Serial.println("[uROS] ABANDON: agent unreachable after 60s");
    vTaskDelete(NULL);
    return;
  }
  Serial.println("[uROS] support_init OK");

  Serial.println("[uROS] Creating node...");
  rclc_node_init_default(&uros_node, "esp32_node", "", &uros_support);
  Serial.println("[uROS] Node OK");

  Serial.println("[uROS] Creating executor...");
  rclc_executor_init(&executor, &uros_support.context, 0, &uros_allocator);
  Serial.println("[uROS] Executor OK");

  // Scan publisher (best effort)
  Serial.println("[uROS] Creating /scan publisher...");
  rclc_publisher_init_best_effort(
      &scan_pub, &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  Serial.println("[uROS] scan_pub OK");

  // IMU publisher (best effort)
  Serial.println("[uROS] Creating /imu publisher...");
  rclc_publisher_init_best_effort(
      &imu_pub, &uros_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu");
  Serial.println("[uROS] imu_pub OK");

  microRosReady = true;
  Serial.println("[uROS] microRosReady = true");

  // Setup LaserScan message
  scan_msg.header.frame_id.data = (char *)"laser";
  scan_msg.header.frame_id.size = strlen("laser");
  scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

  scan_msg.ranges.capacity = 360;
  scan_msg.ranges.size = 360;
  scan_msg.ranges.data = (float *)malloc(360 * sizeof(float));
  if (!scan_msg.ranges.data) {
    Serial.println("[uROS] ERROR: malloc failed for ranges!");
    vTaskDelete(NULL);
    return;
  }

  scan_msg.angle_min = 0.0f;
  scan_msg.angle_max = 2 * M_PI;
  scan_msg.angle_increment = (2 * M_PI) / 360.0f;
  scan_msg.range_min = 0.05f;
  scan_msg.range_max = 12.0f;

  Serial.println("[uROS] Init complete, entering publish loop");

  while (true) {
    // Remplir les ranges depuis le pointBuffer
    for (int i = 0; i < 360; i++)
      scan_msg.ranges.data[i] = 0.0f; // 0 = pas de mesure (ROS convention)

    if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      unsigned long now = millis();
      for (int i = 0; i < pointsAvailable; i++) {
        int idx =
            (pointWriteIndex - 1 - i + POINT_BUFFER_SIZE) % POINT_BUFFER_SIZE;
        LidarPoint &p = pointBuffer[idx];

        // Ignorer les points trop vieux (> 500ms)
        if (now - p.ts > 500)
          continue;
        if (p.confidence < 100)
          continue;

        int a = (int)p.angle;
        if (a >= 0 && a < 360) {
          float dist_m = p.distance * 0.001f;
          // Garder la distance la plus proche pour chaque angle
          if (scan_msg.ranges.data[a] == 0.0f ||
              dist_m < scan_msg.ranges.data[a]) {
            scan_msg.ranges.data[a] = dist_m;
          }
        }
      }
      xSemaphoreGive(bufferMutex);
    }

    // Timestamp
    unsigned long ms = rmw_uros_epoch_millis();
    scan_msg.header.stamp.sec = ms / 1000;
    scan_msg.header.stamp.nanosec = (ms % 1000) * 1000000;

    rcl_publish(&scan_pub, &scan_msg, NULL);

    // Publier IMU
    imu_msg.header.stamp.sec = ms / 1000;
    imu_msg.header.stamp.nanosec = (ms % 1000) * 1000000;
    rcl_publish(&imu_pub, &imu_msg, NULL);

    // Heartbeat
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}
