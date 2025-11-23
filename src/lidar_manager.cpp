#include "lidar_manager.h"
#include "config.h"
#include "globals.h"

HardwareSerial lidarSerial(1);

// Paramètres de sécurité ajustés
#define DIST_STOP_MM 350       // Distance d'arrêt (plus large pour réactivité)
#define OBSTACLE_HOLD_MS 400   // Temps de maintien de l'arrêt (anti-accoups)

void initLidar() {
  // RX sur 16, TX sur 18 (Comme ta config)
  lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("LiDAR initialized (Fast Mode)");
}

// Fonction utilitaire locale
void polarToCartesianLocal(float angle_deg, uint16_t distance_mm, float &x_out, float &y_out) {
  const float RAD = 0.017453292519943295f;
  float a = angle_deg * RAD;
  float meters = (float)distance_mm * 0.001f;
  x_out = sinf(a) * meters;
  y_out = cosf(a) * meters;
}

void lidarTask(void *pvParameters) {
  uint8_t buf[PACKET_SIZE];
  unsigned long lastObstacleTime = 0;
  
  while (true) {
    if (lidarSerial.available()) {
      // Recherche de l'en-tête 0x54
      if (lidarSerial.read() == 0x54) {
        // On attend d'avoir le reste du paquet (optimisation blockante courte)
        if (lidarSerial.readBytes(buf, PACKET_SIZE - 1) == (PACKET_SIZE - 1)) {
          
          // Vérification rapide du type de paquet (0x2C pour LD06/LD19)
          if (buf[0] == 0x2C) {
            
            // Parsing des angles
            float start_angle = (buf[3] | (buf[4] << 8)) / 100.0;
            float end_angle = (buf[41] | (buf[42] << 8)) / 100.0;
            if (end_angle < start_angle) end_angle += 360.0;
            float step = (end_angle - start_angle) / (MEAS_PER_PACKET - 1);
            
            // Vitesse de rotation (pour info)
            uint16_t speed = buf[1] | (buf[2] << 8);
            currentRotationSpeed = speed / 360.0; // Hz approx

            bool localObstacleFound = false;
            uint16_t localMinDist = 9999;

            // --- SECTION CRITIQUE (Buffer Points) ---
            if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
              
              for (int i = 0; i < MEAS_PER_PACKET; i++) {
                int base = 5 + (i * 3);
                uint16_t dist = buf[base] | (buf[base + 1] << 8);
                uint8_t conf = buf[base + 2];

                if (conf > 100 && dist > 0 && dist < 12000) { // Confiance min > 100 pour éviter le bruit
                  float ang = start_angle + step * i;
                  if (ang >= 360.0) ang -= 360.0;

                  // Mise à jour Buffer pour Streaming
                  LidarPoint point;
                  point.angle = ang;
                  point.distance = dist;
                  point.confidence = conf;
                  polarToCartesianLocal(ang, dist, point.x, point.y);
                  point.ts = millis();
                  
                  pointBuffer[pointWriteIndex] = point;
                  pointWriteIndex = (pointWriteIndex + 1) % POINT_BUFFER_SIZE;
                  if (pointsAvailable < POINT_BUFFER_SIZE) pointsAvailable++;
                  
                  // --- LOGIQUE DE DÉTECTION ---
                  // Secteur Avant : 330° à 360° OU 0° à 30°
                  bool inFront = (ang >= FRONT_ANGLE_MIN || ang <= FRONT_ANGLE_MAX);
                  
                  if (inFront && dist < DIST_STOP_MM) {
                     localObstacleFound = true;
                     if (dist < localMinDist) localMinDist = dist;
                  }
                }
              }
              xSemaphoreGive(bufferMutex);
            }
            // ----------------------------------------

            // --- GESTION ÉTAT OBSTACLE (HYSTÉRÉSIS) ---
            // C'est ici que la magie opère pour supprimer les accoups
            
            if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
              if (localObstacleFound) {
                // Si on voit un obstacle, on le marque et on reset le timer
                obstacleDetected = true;
                minObstacleDistance = localMinDist;
                lastObstacleTime = millis();
              } 
              else {
                // Si on ne voit PLUS d'obstacle, on attend 400ms avant de relâcher
                // Cela empêche le robot de redémarrer si l'obstacle clignote
                if (obstacleDetected && (millis() - lastObstacleTime > OBSTACLE_HOLD_MS)) {
                  obstacleDetected = false;
                  minObstacleDistance = 9999;
                }
              }
              lastLidarUpdate = millis();
              xSemaphoreGive(lidarMutex);
            }
          }
        }
      }
    } else {
      // Si pas de données, on laisse la main un court instant
      vTaskDelay(1);
    }
  }
}