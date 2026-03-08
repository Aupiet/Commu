#include "lidar_manager.h"
#include "config.h"
#include "globals.h"

HardwareSerial lidarSerial(1);

void initLidar() {
  lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("LiDAR initialized");
}

uint8_t crc8_compute(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x4D);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void polarToCartesian(float angle_deg, uint16_t distance_mm, float &x_out, float &y_out) {
  const float RAD = 0.017453292519943295f;
  float a = angle_deg * RAD;
  float meters = (float)distance_mm * 0.001f;
  x_out = sinf(a) * meters;
  y_out = cosf(a) * meters;
}

void lidarReadTask(void *pvParameters) {
  uint8_t buf[PACKET_SIZE];
  uint8_t idx = 0;
  unsigned long lastByteTime = 0;
  
  while (true) {
    if (lidarSerial.available()) {
      uint8_t b = lidarSerial.read();
      unsigned long now = micros();
      
      if (idx == 0) {
        if (b == 0x54) {
          buf[idx++] = b;
          lastByteTime = now;
        }
      } else if (idx == 1) {
        buf[idx++] = b;
        lastByteTime = now;
      } else {
        if (micros() - lastByteTime > 2000) {
          idx = 0;
          continue;
        }
        buf[idx++] = b;
        lastByteTime = now;
      }
      
      if (idx >= PACKET_SIZE) {
        if (buf[0] == 0x54 && buf[1] == 0x2C) {
          uint8_t crc = crc8_compute(buf, PACKET_SIZE - 1);
          if (crc == buf[PACKET_SIZE - 1]) {
            if (xQueueSend(packetQueue, buf, 0) == pdTRUE) {
              packetsReceived++;
            }
          }
        }
        idx = 0;
      }
    } else {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

void lidarProcessTask(void *pvParameters) {
  uint8_t raw[PACKET_SIZE];
  LD06Packet p;
  
  while (true) {
    if (xQueueReceive(packetQueue, raw, portMAX_DELAY) == pdTRUE) {
      memcpy(&p, raw, sizeof(raw));
      
      currentRotationSpeed = (float)p.speed * 0.01f;
      float start_angle = (float)p.start_angle * 0.01f;
      float end_angle = (float)p.end_angle * 0.01f;
      if (end_angle < start_angle) end_angle += 360.0f;
      float step = (end_angle - start_angle) / (MEAS_PER_PACKET - 1);
      
      bool obstacleNow = false;
      uint16_t minDist = 9999;
      
      if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < MEAS_PER_PACKET; ++i) {
          uint16_t dist = p.distances[i];
          uint8_t conf = p.confidences[i];
          
          if (conf > 0 && dist > 0 && dist < 12000) {
            float ang = start_angle + step * i;
            if (ang >= 360.0f) ang -= 360.0f;
            
            LidarPoint point;
            point.angle = ang;
            point.distance = dist;
            point.confidence = conf;
            polarToCartesian(ang, dist, point.x, point.y);
            point.ts = millis();
            
            pointBuffer[pointWriteIndex] = point;
            pointWriteIndex = (pointWriteIndex + 1) % POINT_BUFFER_SIZE;
            if (pointsAvailable < POINT_BUFFER_SIZE) {
              pointsAvailable++;
            } else {
              pointReadIndex = (pointReadIndex + 1) % POINT_BUFFER_SIZE;
            }
            totalPointsProcessed++;
            
            bool inFrontSector = (ang >= FRONT_ANGLE_MIN && ang <= 360.0f) ||
                                 (ang >= 0.0f && ang <= FRONT_ANGLE_MAX);
            
            if (inFrontSector) {
              if (dist < minDist) minDist = dist;
              if (dist < OBSTACLE_DISTANCE) {
                obstacleNow = true;
              }
            }
          }
        }
        xSemaphoreGive(bufferMutex);
      }
      
      if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        obstacleDetected = obstacleNow;
        minObstacleDistance = minDist;
        lastLidarUpdate = millis();
        xSemaphoreGive(lidarMutex);
      }
    }
  }
}