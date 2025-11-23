#ifndef LIDAR_MANAGER_H
#define LIDAR_MANAGER_H

#include <Arduino.h>

void initLidar();
// Une seule tâche optimisée qui lit ET traite
void lidarTask(void *pvParameters); 

#endif