#ifndef LIDAR_MANAGER_H
#define LIDAR_MANAGER_H

#include <Arduino.h>

void initLidar();
uint8_t crc8_compute(const uint8_t *data, size_t len);
void polarToCartesian(float angle_deg, uint16_t distance_mm, float &x_out, float &y_out);
void lidarReadTask(void *pvParameters);
void lidarProcessTask(void *pvParameters);

#endif