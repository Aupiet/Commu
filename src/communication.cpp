#include "communication.h"
#include "config.h"
#include "globals.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include "speed_estimator.h" 

void initCommunication() {
  WiFi.mode(WIFI_AP_STA);
  // On force le canal 1 pour la stabilité ESP-NOW
  // Cela évite que l'ESP32 change de canal et perde l'ESP-01
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, 0);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1);
  }
  
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp01Address, 6);
  peerInfo.channel = 1; // Canal fixe (doit matcher le softAP)
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("ESP-01 peer added");
  }
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg = "";
  for (int i = 0; i < len; i++) msg += (char)incomingData[i];
  
  JsonDocument doc;
  if (deserializeJson(doc, msg) == DeserializationError::Ok && doc["T"] == 1) {
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      float l = doc["L"] | 0.0;
      float r = doc["R"] | 0.0;
      ctrlData.joyY = (int)((l + r) / 2.0 * 512);
      ctrlData.joyX = (int)((r - l) / 2.0 * 512);
      ctrlData.timestamp = millis();
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
  static unsigned long lastSpeed = 0;
  static unsigned long lastLidar = 0;

  while (true) {
    // Drapeau pour éviter d'envoyer 2 paquets en même temps (Collision)
    // Si on envoie un paquet dans ce tour de boucle, on passe le tour pour l'autre
    bool packetSentThisLoop = false; 
    unsigned long now = millis();

    // --- 1. Préparation Données ---
    bool obs = false;
    float dist = 0;
    if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      obs = obstacleDetected;
      dist = minObstacleDistance;
      xSemaphoreGive(lidarMutex);
    }
    
    float speed = estimatedSpeed.speed_m_per_sec;
    int rssi = WiFi.RSSI();

    // --- 2. Envoi VITESSE (Type 130) ---
    // Prioritaire : On l'envoie toutes les 200ms
    if (now - lastSpeed > 200 && !packetSentThisLoop) {
      String jsonSpd = "{\"T\":130,\"L\":" + String(speed, 2) + 
                       ",\"R\":" + String(speed, 2) + 
                       ",\"V\":12.0,\"RSSI\":" + String(rssi) + "}";
      
      esp_err_t result = esp_now_send(esp01Address, (uint8_t*)jsonSpd.c_str(), jsonSpd.length());
      
      if (result == ESP_OK) {
        lastSpeed = now;
        packetSentThisLoop = true; // Bloque l'envoi LiDAR pour ce tour de boucle
      }
    }

    // --- 3. Envoi LIDAR (Type 2000) ---
    // S'exécute toutes les 100ms, MAIS SEULEMENT si la voie est libre (!packetSentThisLoop)
    if (now - lastLidar > 100 && !packetSentThisLoop) {
      String jsonLidar = "{\"T\":2000,\"dist\":" + String(dist, 1) + 
                         ",\"obstacle\":" + String(obs ? "true" : "false") + "}";
      
      esp_err_t result = esp_now_send(esp01Address, (uint8_t*)jsonLidar.c_str(), jsonLidar.length());
      
      if (result == ESP_OK) {
        lastLidar = now;
        packetSentThisLoop = true;
      }
    }
    
    // Pause plus courte (20ms) pour revenir vite vérifier si on peut envoyer le paquet en attente
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void streamingTask(void *pvParameters) {
  streamServer.begin();
  
  while (true) {
    if (!streamClient || !streamClient.connected()) {
      WiFiClient c = streamServer.available();
      if (c) streamClient = c;
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