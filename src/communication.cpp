#include "communication.h"
#include "config.h"
#include "globals.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
// Ajout pour la fusion de vitesse
#include "speed_estimator.h" 

void initCommunication() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi AP: ");
  Serial.println(WiFi.softAPIP());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1);
  }
  
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp01Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("ESP-01 peer added");
  }
  
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg = "";
  for (int i = 0; i < len; i++) {
    msg += (char)incomingData[i];
  }
  
  JsonDocument doc;
  if (deserializeJson(doc, msg) == DeserializationError::Ok && doc["T"] == 1) {
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      float left = doc["L"] | 0.0;
      float right = doc["R"] | 0.0;
      
      float forward = (left + right) / 2.0;
      float turn = (right - left) / 2.0;
      
      ctrlData.joyY = (int)(forward * 512);
      ctrlData.joyX = (int)(turn * 512);
      ctrlData.button = 0;
      
      receivedCommands++;
      lastCommandTime = millis();
      xSemaphoreGive(ctrlMutex);
    }
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    sentFeedback++;
  }
}

void communicationTask(void *pvParameters) {
  while (true) {
    bool obstacle = false;
    uint16_t distance = 9999;
    
    if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      obstacle = obstacleDetected;
      distance = minObstacleDistance;
      xSemaphoreGive(lidarMutex);
    }
    
    // --- MODIFICATION : Utilisation de la vitesse fusionnée ---
    // On récupère la vitesse estimée en m/s
    float speed = abs(estimatedSpeed.speed_m_per_sec); 
    Serial.println("Estimated speed (m/s): " + String(estimatedSpeed.speed_m_per_sec, 2));
    // Fallback de sécurité : si l'estimateur n'a jamais tourné (timestamp à 0),
    // on utilise l'ancienne méthode basée sur le PWM
    if (estimatedSpeed.last_update_ms == 0) {
         speed = abs(currentSpeedPWM) * 0.01f; 
    }
    
    int rssi = WiFi.RSSI();
    
    if (millis() - lastFeedbackTime > 100) {
      // Format strict conservé : {"T":130, "L":vitesse, "R":vitesse, ...}
      String json130 = "{\"T\":130,\"L\":" + String(speed, 2) + 
                       ",\"R\":" + String(speed, 2) + 
                       ",\"V\":11.7,\"RSSI\":" + String(rssi) + "}";
      Serial.println("Sending feedback: " + json130);
      esp_now_send(esp01Address, (uint8_t*)json130.c_str(), json130.length());
      lastFeedbackTime = millis();
    }
    
    static unsigned long lastLidar = 0;
    if (millis() - lastLidar > 200) {
      String json2000 = "{\"T\":2000,\"dist\":" + String(distance / 10.0, 1) + 
                        ",\"obstacle\":" + String(obstacle ? "true" : "false") + "}";
      
      esp_now_send(esp01Address, (uint8_t*)json2000.c_str(), json2000.length());
      lastLidar = millis();
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void streamingTask(void *pvParameters) {
  streamServer.begin();
  
  while (true) {
    if (!streamClient || !streamClient.connected()) {
      WiFiClient c = streamServer.available();
      if (c) {
        streamClient = c;
        Serial.println("Client connected: " + streamClient.remoteIP().toString());
      }
    }
    
    if (streamClient && streamClient.connected()) {
      String line = "[";
      bool first = true;
      
      if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int pts = std::min((int)pointsAvailable, 200);
        unsigned long now = millis();
        
        for (int i = 0; i < pts; ++i) {
          int idx = (pointReadIndex + i) % POINT_BUFFER_SIZE;
          LidarPoint &p = pointBuffer[idx];
          
          if (now - p.ts < 3000) {
            if (!first) line += ",";
            line += "[" + String(p.angle, 1) + "," + 
                    String(p.distance) + "," + 
                    String(p.confidence) + "," +
                    String(p.x, 4) + "," + 
                    String(p.y, 4) + "]";
            first = false;
          }
        }
        xSemaphoreGive(bufferMutex);
      }
      
      line += "]\n";
      streamClient.print(line);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}