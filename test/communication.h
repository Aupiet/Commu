#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <esp_now.h>

void initCommunication();
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void communicationTask(void *pvParameters);
void streamingTask(void *pvParameters);

#endif