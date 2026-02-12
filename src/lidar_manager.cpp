#include "lidar_manager.h"
#include "config.h"
#include "globals.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>

HardwareSerial lidarSerial(1);

// ===== micro-ROS =====
static rcl_publisher_t scan_pub;
static rcl_publisher_t obstacle_pub;
static sensor_msgs__msg__LaserScan scan_msg;
static std_msgs__msg__Bool obstacle_msg;

// Partagés avec imuTask (non-static)
rcl_node_t uros_node;
rclc_support_t uros_support;
rcl_allocator_t uros_allocator;
volatile bool microRosReady = false; // Signal pour les autres tâches

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

// ===== TÂCHE LiDAR TEMPS RÉEL (INCHANGÉE) =====
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

// ===== TÂCHE micro-ROS (SOFT REALTIME) =====
void microRosLidarTask(void *pv) {

  Serial.println("[uROS-LIDAR] Task started, waiting 2s...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  rclc_executor_t executor;
  uros_allocator = rcl_get_default_allocator();

  // Retry loop pour rclc_support_init (attend que l'agent soit joignable)
  Serial.println("[uROS-LIDAR] Calling rclc_support_init...");
  rcl_ret_t ret;
  int attempts = 0;
  do {
    ret = rclc_support_init(&uros_support, 0, NULL, &uros_allocator);
    if (ret != RCL_RET_OK) {
      attempts++;
      Serial.printf("[uROS-LIDAR] support_init FAILED (ret=%d), attempt %d, "
                    "retrying in 2s...\n",
                    (int)ret, attempts);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  } while (ret != RCL_RET_OK && attempts < 30);

  if (ret != RCL_RET_OK) {
    Serial.println("[uROS-LIDAR] ABANDON: agent unreachable after 60s");
    vTaskDelete(NULL);
    return;
  }
  Serial.println("[uROS-LIDAR] support_init OK");

  Serial.println("[uROS-LIDAR] Creating node...");
  rclc_node_init_default(&uros_node, "esp32_node", "", &uros_support);
  Serial.println("[uROS-LIDAR] Node OK");

  Serial.println("[uROS-LIDAR] Creating executor...");
  rclc_executor_init(&executor, &uros_support.context, 0, &uros_allocator);
  Serial.println("[uROS-LIDAR] Executor OK");

  Serial.println("[uROS-LIDAR] Creating scan publisher...");
  rclc_publisher_init_default(
      &scan_pub, &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  Serial.println("[uROS-LIDAR] scan_pub OK");

  Serial.println("[uROS-LIDAR] Creating obstacle publisher...");
  rclc_publisher_init_default(&obstacle_pub, &uros_node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                              "/obstacle");
  Serial.println("[uROS-LIDAR] obstacle_pub OK");

  // Signaler que micro-ROS est prêt (pour imuTask)
  microRosReady = true;
  Serial.println("[uROS-LIDAR] microRosReady = true");

  scan_msg.header.frame_id.data = (char *)"laser";
  scan_msg.header.frame_id.size = strlen("laser");
  scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

  // Préallocation LaserScan
  scan_msg.ranges.capacity = 360;
  scan_msg.ranges.size = 360;
  scan_msg.ranges.data = (float *)malloc(360 * sizeof(float));

  scan_msg.angle_min = 0.0f;
  scan_msg.angle_max = 2 * M_PI;
  scan_msg.angle_increment = (2 * M_PI) / 360.0f;
  scan_msg.range_min = 0.05f;
  scan_msg.range_max = 12.0f;

  Serial.println("[uROS-LIDAR] Init complete, entering publish loop");

  while (true) {

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

    obstacle_msg.data = obstacleDetected;

    scan_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    scan_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;

    rcl_publish(&scan_pub, &scan_msg, NULL);
    rcl_publish(&obstacle_pub, &obstacle_msg, NULL);

    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}
