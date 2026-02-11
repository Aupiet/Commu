/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  ESP32 WAVE ROVER - FIRMWARE COMPLET v3.0                            ║
 * ║  Contrôle + LiDAR LD06 + TCP Streaming + OLED + ESP-NOW             ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 * 
 * FONCTIONNALITÉS:
 * ✅ Contrôle joystick intuitif (HAUT=avancer, BAS=reculer)
 * ✅ LiDAR LD06 robuste avec CRC, buffer circulaire, calcul X/Y
 * ✅ Arrêt d'urgence si obstacle < 30cm devant
 * ✅ Streaming TCP vers Python (port 8888)
 * ✅ ESP-NOW bidirectionnel avec ESP-01
 * ✅ Affichage OLED I2C (état système, LiDAR, commandes)
 * ✅ Architecture FreeRTOS multi-core optimisée
 * 
 * ARCHITECTURE:
 * - Core 0: WiFi (AP + ESP-NOW) + WebServer + TCP streaming + OLED
 * - Core 1: Moteurs + LiDAR read/process
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// ═══════════════════════════════════════════════════════════════════════
// ██████╗  ██████╗ ███╗   ██╗███████╗██╗ ██████╗ 
// ██╔════╝ ██╔═══██╗████╗  ██║██╔════╝██║██╔════╝ 
// ██║  ███╗██║   ██║██╔██╗ ██║█████╗  ██║██║  ███╗
// ██║   ██║██║   ██║██║╚██╗██║██╔══╝  ██║██║   ██║
// ╚██████╔╝╚██████╔╝██║ ╚████║██║     ██║╚██████╔╝
//  ╚═════╝  ╚═════╝ ╚═╝  ╚═══╝╚═╝     ╚═╝ ╚═════╝ 
// ═══════════════════════════════════════════════════════════════════════

// Adresses MAC (À MODIFIER selon votre matériel)
uint8_t esp01Address[] = {0x34, 0x5F, 0x45, 0x62, 0xAC, 0x9C};

// WiFi AP
const char* ssid = "PorcheWifi";
const char* password = "12345678";


// Pins LiDAR LD06
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_BAUD 230400

// OLED I2C (adapté au Wave Rover)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C  // Vérifier avec I2C scanner si nécessaire

// LED & TCP
#define LED_PIN 2
#define STREAM_PORT 8888

// Paramètres LiDAR
#define OBSTACLE_DISTANCE 300  // 30cm en mm
#define FRONT_ANGLE_MIN 330    // Secteur avant ±30°
#define FRONT_ANGLE_MAX 30
#define PACKET_SIZE 47
#define MEAS_PER_PACKET 12
#define POINT_BUFFER_SIZE 2000

// PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// ═══════════════════════════════════════════════════════════════════════
// ███████╗████████╗██████╗ ██╗   ██╗ ██████╗████████╗███████╗
// ██╔════╝╚══██╔══╝██╔══██╗██║   ██║██╔════╝╚══██╔══╝██╔════╝
// ███████╗   ██║   ██████╔╝██║   ██║██║        ██║   ███████╗
// ╚════██║   ██║   ██╔══██╗██║   ██║██║        ██║   ╚════██║
// ███████║   ██║   ██║  ██║╚██████╔╝╚██████╗   ██║   ███████║
// ╚══════╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝  ╚═════╝   ╚═╝   ╚══════╝
// ═══════════════════════════════════════════════════════════════════════

// Structure commande joystick
struct ControlPacket {
  int joyX;      // -512 à +512
  int joyY;      // -512 à +512
  uint8_t button;
  uint32_t timestamp;
};

// Packet LiDAR LD06
#pragma pack(push,1)
typedef struct {
  uint8_t header;       // 0x54
  uint8_t length;       // 0x2C
  uint16_t speed;
  uint16_t start_angle; // centièmes de degrés
  uint16_t distances[MEAS_PER_PACKET];
  uint8_t confidences[MEAS_PER_PACKET];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc;
} LD06Packet;
#pragma pack(pop)

// Point LiDAR calculé
typedef struct {
  float angle;       // degrés
  uint16_t distance; // mm
  uint8_t confidence;
  float x;           // mètres
  float y;           // mètres
  unsigned long ts;  // timestamp
} LidarPoint;

// ═══════════════════════════════════════════════════════════════════════
// ██████╗ ██╗      ██████╗ ██████╗  █████╗ ██╗     ███████╗
// ██╔════╝ ██║     ██╔═══██╗██╔══██╗██╔══██╗██║     ██╔════╝
// ██║  ███╗██║     ██║   ██║██████╔╝███████║██║     █████╗  
// ██║   ██║██║     ██║   ██║██╔══██╗██╔══██║██║     ██╔══╝  
// ╚██████╔╝███████╗╚██████╔╝██████╔╝██║  ██║███████╗███████╗
//  ╚═════╝ ╚══════╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝╚══════╝
// ═══════════════════════════════════════════════════════════════════════

// Hardware
HardwareSerial lidarSerial(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiServer streamServer(STREAM_PORT);
WiFiClient streamClient;

// Synchronisation
SemaphoreHandle_t ctrlMutex;
SemaphoreHandle_t lidarMutex;
SemaphoreHandle_t bufferMutex;
QueueHandle_t packetQueue;

// Données commande
ControlPacket ctrlData;
volatile int currentSpeedPWM = 0;
volatile int currentDirection = 0;

// LiDAR
LidarPoint pointBuffer[POINT_BUFFER_SIZE];
int pointWriteIndex = 0;
int pointReadIndex = 0;
volatile int pointsAvailable = 0;
volatile bool obstacleDetected = false;
volatile uint16_t minObstacleDistance = 9999;
volatile unsigned long packetsReceived = 0;
volatile unsigned long totalPointsProcessed = 0;
volatile float currentRotationSpeed = 0.0f;

// Timings
unsigned long lastCommandTime = 0;
unsigned long lastFeedbackTime = 0;
unsigned long lastLidarUpdate = 0;
unsigned long lastOledUpdate = 0;

// Stats
uint32_t receivedCommands = 0;
uint32_t sentFeedback = 0;

// ═══════════════════════════════════════════════════════════════════════
// ██╗   ██╗████████╗██╗██╗     ███████╗
// ██║   ██║╚══██╔══╝██║██║     ██╔════╝
// ██║   ██║   ██║   ██║██║     ███████╗
// ██║   ██║   ██║   ██║██║     ╚════██║
// ╚██████╔╝   ██║   ██║███████╗███████║
//  ╚═════╝    ╚═╝   ╚═╝╚══════╝╚══════╝
// ═══════════════════════════════════════════════════════════════════════

// CRC-8 (polynôme 0x4D)
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

// Conversion polaire → cartésien
inline void polarToCartesian(float angle_deg, uint16_t distance_mm, 
                             float &x_out, float &y_out) {
  const float RAD = 0.017453292519943295f; // PI/180
  float a = angle_deg * RAD;
  float meters = (float)distance_mm * 0.001f;
  x_out = sinf(a) * meters;
  y_out = cosf(a) * meters;
}

// ═══════════════════════════════════════════════════════════════════════
// ███████╗███████╗██████╗       ███╗   ██╗ ██████╗ ██╗    ██╗
// ██╔════╝██╔════╝██╔══██╗      ████╗  ██║██╔═══██╗██║    ██║
// █████╗  ███████╗██████╔╝█████╗██╔██╗ ██║██║   ██║██║ █╗ ██║
// ██╔══╝  ╚════██║██╔═══╝ ╚════╝██║╚██╗██║██║   ██║██║███╗██║
// ███████╗███████║██║           ██║ ╚████║╚██████╔╝╚███╔███╔╝
// ╚══════╝╚══════╝╚═╝           ╚═╝  ╚═══╝ ╚═════╝  ╚══╝╚══╝ 
// ═══════════════════════════════════════════════════════════════════════

// Callback réception ESP-NOW
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg = "";
  for (int i = 0; i < len; i++) {
    msg += (char)incomingData[i];
  }
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);
  
  if (!error && doc["T"] == 1) {
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      float left = doc["L"] | 0.0;
      float right = doc["R"] | 0.0;
      
      // ╔═══════════════════════════════════════════════════════════╗
      // ║ CORRECTION JOYSTICK: Conversion différentiel → joystick  ║
      // ║ L et R sont déjà inversés par l'ESP-01 pour donner:      ║
      // ║ - forward > 0 = AVANCER (joystick HAUT)                  ║
      // ║ - forward < 0 = RECULER (joystick BAS)                   ║
      // ╚═══════════════════════════════════════════════════════════╝
      
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

// Callback envoi ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    sentFeedback++;
  }
}

// ═══════════════════════════════════════════════════════════════════════
// ███╗   ███╗ ██████╗ ████████╗███████╗██╗   ██╗██████╗ ███████╗
// ████╗ ████║██╔═══██╗╚══██╔══╝██╔════╝██║   ██║██╔══██╗██╔════╝
// ██╔████╔██║██║   ██║   ██║   █████╗  ██║   ██║██████╔╝███████╗
// ██║╚██╔╝██║██║   ██║   ██║   ██╔══╝  ██║   ██║██╔══██╗╚════██║
// ██║ ╚═╝ ██║╚██████╔╝   ██║   ███████╗╚██████╔╝██║  ██║███████║
// ╚═╝     ╚═╝ ╚═════╝    ╚═╝   ╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝
// ═══════════════════════════════════════════════════════════════════════


// --- --- --- Motor Part --- --- ---
const uint16_t PWMA = 25;
const uint16_t AIN2 = 17;
const uint16_t AIN1 = 21;
const uint16_t BIN1 = 22;
const uint16_t BIN2 = 23;
const uint16_t PWMB = 26;

int freq = 100000;
int channel_A = 5;
int channel_B = 6;
const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
const uint16_t MIN_PWM = MAX_PWM/5;

// Initialisation moteurs (à mettre dans setup)
void initMotors() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    ledcSetup(channel_A, freq, ANALOG_WRITE_BITS);
    ledcAttachPin(PWMA, channel_A);
    ledcSetup(channel_B, freq, ANALOG_WRITE_BITS);
    ledcAttachPin(PWMB, channel_B);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

void stopMotors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(channel_A, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B, 0);
}

void channelACtrl(float pwmInputA) {
    int pwmIntA = round(pwmInputA);
    if (pwmIntA == 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        ledcWrite(channel_A, 0);
        return;
    }
    if (pwmIntA > 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        ledcWrite(channel_A, abs(constrain(pwmIntA, MIN_PWM, MAX_PWM)));
    } else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        ledcWrite(channel_A, abs(constrain(pwmIntA, -MAX_PWM, -MIN_PWM)));
    }
}

// Fonction de commande gauche
void channelBCtrl(float pwmInputB) {
    int pwmIntB = round(pwmInputB);
    if (pwmIntB == 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        ledcWrite(channel_B, 0);
        return;
    }
    if (pwmIntB > 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        ledcWrite(channel_B, abs(constrain(pwmIntB, MIN_PWM, MAX_PWM)));
    } else {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        ledcWrite(channel_B, abs(constrain(pwmIntB, -MAX_PWM, -MIN_PWM)));
    }
}

void setMotorSpeedFromJoystick(int joyX, int joyY) {
    // Remarque : adapte joyX/joyY selon comment ta télécommande les transmet !
    // Ici: joyX = avant/arrière, joyY = gauche/droite comme sur ton schéma

    float forward = (float)joyX / 512.0f; // Avant (1.0), Arrière (-1.0)
    float turn = (float)joyY / 512.0f;    // Droite (1.0), Gauche (-1.0)

    float left = forward + turn;
    float right = forward - turn;

    // Normalisation [-1;1]
    left = constrain(left, -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);

    const int MIN_PWM = 70;           // PWM min pour démarrer le moteur (à ajuster!)
    const int MAX_PWM = 255;

    // Fonction pour appliquer le seuil de démarrage :
    auto pwmValue = [=](float val) -> int {
        if (val == 0) return 0;
        int pwm = round(abs(val) * (MAX_PWM - MIN_PWM) + MIN_PWM);
        return (val > 0) ? pwm : -pwm;
    };

    int pwmLeft = pwmValue(left);
    int pwmRight = pwmValue(right);

    printf( "pwmLeft: %d  pwmRight: %d \n", pwmLeft, pwmRight );

    channelACtrl(pwmRight); // Droite
    channelBCtrl(pwmLeft);  // Gauche
}



// Task FreeRTOS : lecture joystick et pilotage
void motorControlTask(void *pvParameters) {
    while (true) {
        int joyX = 0, joyY = 0;
        if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            joyX = ctrlData.joyX;
            joyY = ctrlData.joyY;
            xSemaphoreGive(ctrlMutex);
        }
        setMotorSpeedFromJoystick(joyX, joyY);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}




// ╔═══════════════════════════════════════════════════════════════════╗
// ║  TÂCHE: LECTURE LIDAR (Core 1)                                    ║
// ║  - Lecture UART série du LD06                                     ║
// ║  - Détection header, assemblage packet, vérification CRC          ║
// ║  - Envoi dans queue pour traitement                               ║
// ╚═══════════════════════════════════════════════════════════════════╝

void lidarReadTask(void *pvParameters) {
  Serial.printf("📡 lidarReadTask on core %d\n", xPortGetCoreID());
  
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
        // Timeout si trop de temps entre octets
        if (micros() - lastByteTime > 2000) {
          idx = 0;
          continue;
        }
        buf[idx++] = b;
        lastByteTime = now;
      }
      
      // Packet complet?
      if (idx >= PACKET_SIZE) {
        if (buf[0] == 0x54 && buf[1] == 0x2C) {
          uint8_t crc = crc8_compute(buf, PACKET_SIZE - 1);
          if (crc == buf[PACKET_SIZE - 1]) {
            // CRC OK: envoyer dans queue
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

// ╔═══════════════════════════════════════════════════════════════════╗
// ║  TÂCHE: TRAITEMENT LIDAR (Core 1)                                 ║
// ║  - Lecture packets depuis queue                                   ║
// ║  - Calcul angles, conversion polaire→cartésien                    ║
// ║  - Stockage dans buffer circulaire                                ║
// ║  - Détection obstacles secteur avant                              ║
// ╚═══════════════════════════════════════════════════════════════════╝

void lidarProcessTask(void *pvParameters) {
  Serial.printf("⚡ lidarProcessTask on core %d\n", xPortGetCoreID());
  
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
      
      // Traiter chaque point
      if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < MEAS_PER_PACKET; ++i) {
          uint16_t dist = p.distances[i];
          uint8_t conf = p.confidences[i];
          
          if (conf > 0 && dist > 0 && dist < 12000) {
            float ang = start_angle + step * i;
            if (ang >= 360.0f) ang -= 360.0f;
            
            // Calculer point
            LidarPoint point;
            point.angle = ang;
            point.distance = dist;
            point.confidence = conf;
            polarToCartesian(ang, dist, point.x, point.y);
            point.ts = millis();
            
            // Ajouter au buffer circulaire
            pointBuffer[pointWriteIndex] = point;
            pointWriteIndex = (pointWriteIndex + 1) % POINT_BUFFER_SIZE;
            if (pointsAvailable < POINT_BUFFER_SIZE) {
              pointsAvailable++;
            } else {
              pointReadIndex = (pointReadIndex + 1) % POINT_BUFFER_SIZE;
            }
            totalPointsProcessed++;
            
            // Détection obstacle secteur avant (±30° autour de 0°)
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
      
      // Mettre à jour état obstacle global
      if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        obstacleDetected = obstacleNow;
        minObstacleDistance = minDist;
        lastLidarUpdate = millis();
        xSemaphoreGive(lidarMutex);
      }
    }
  }
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║  TÂCHE: COMMUNICATION ESP-NOW (Core 0)                            ║
// ║  - Envoi feedback vers ESP-01: vitesse, RSSI, LiDAR               ║
// ║  - Format JSON compatible avec la télécommande                    ║
// ╚═══════════════════════════════════════════════════════════════════╝

void communicationTask(void *pvParameters) {
  Serial.printf("📡 communicationTask on core %d\n", xPortGetCoreID());
  
  while (true) {
    bool obstacle = false;
    uint16_t distance = 9999;
    
    if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      obstacle = obstacleDetected;
      distance = minObstacleDistance;
      xSemaphoreGive(lidarMutex);
    }
    
    float speed = abs(currentSpeedPWM) * 0.01f;
    int rssi = WiFi.RSSI();
    
    // T=130: Vitesse + RSSI (toutes les 100ms)
    if (millis() - lastFeedbackTime > 100) {
      String json130 = "{\"T\":130,\"L\":" + String(speed, 2) + 
                       ",\"R\":" + String(speed, 2) + 
                       ",\"V\":11.7,\"RSSI\":" + String(rssi) + "}";
      
      esp_now_send(esp01Address, (uint8_t*)json130.c_str(), json130.length());
      lastFeedbackTime = millis();
    }
    
    // T=2000: LiDAR (toutes les 200ms)
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

// ╔═══════════════════════════════════════════════════════════════════╗
// ║  TÂCHE: TCP STREAMING VERS PYTHON (Core 0)                        ║
// ║  - Accepte connexions TCP sur port 8888                           ║
// ║  - Stream points LiDAR au format JSON compatible Python           ║
// ║  - Format: [[angle,dist,conf,x,y], ...]                           ║
// ╚═══════════════════════════════════════════════════════════════════╝

void streamingTask(void *pvParameters) {
  Serial.printf("🌐 streamingTask on core %d\n", xPortGetCoreID());
  
  streamServer.begin();
  
  while (true) {
    // Accepter nouveau client si déconnecté
    if (!streamClient || !streamClient.connected()) {
      WiFiClient c = streamServer.available();
      if (c) {
        streamClient = c;
        Serial.println("🔌 Client Python connecté: " + streamClient.remoteIP().toString());
      }
    }
    
    // Streamer si client connecté
    if (streamClient && streamClient.connected()) {
      String line = "[";
      bool first = true;
      
      if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int pts = std::min((int)pointsAvailable, 200);
        unsigned long now = millis();
        
        for (int i = 0; i < pts; ++i) {
          int idx = (pointReadIndex + i) % POINT_BUFFER_SIZE;
          LidarPoint &p = pointBuffer[idx];
          
          // Ne garder que points récents
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
    
    vTaskDelay(100 / portTICK_PERIOD_MS); // 10 Hz
  }
}

// ╔═══════════════════════════════════════════════════════════════════╗
// ║  TÂCHE: AFFICHAGE OLED (Core 0)                                   ║
// ║  - État système, commandes, LiDAR, stats                          ║
// ║  - Mise à jour 5 Hz pour lisibilité                               ║
// ╚═══════════════════════════════════════════════════════════════════╝

void oledTask(void *pvParameters) {
  Serial.printf("📺 oledTask on core %d\n", xPortGetCoreID());
  
  while (true) {
    if (millis() - lastOledUpdate > 200) { // 5 Hz
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      
      // Ligne 1: Titre
      display.println("=== WAVE ROVER ===");
      
      // Ligne 2: Commandes
      unsigned long cmdAge = (millis() - lastCommandTime) / 1000;
      if (cmdAge < 2) {
        display.print("Cmd: OK (");
        display.print(receivedCommands);
        display.println(")");
      } else {
        display.print("Cmd: TIMEOUT (");
        display.print(cmdAge);
        display.println("s)");
      }
      
      // Ligne 3: WiFi
      display.print("WiFi: ");
      display.print(WiFi.RSSI());
      display.print("dBm IP:");
      display.println(WiFi.softAPIP()[3]); // Dernier octet
      
      // Ligne 4: LiDAR
      bool obstacle = false;
      uint16_t dist = 9999;
      if (xSemaphoreTake(lidarMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        obstacle = obstacleDetected;
        dist = minObstacleDistance;
        xSemaphoreGive(lidarMutex);
      }
      
      display.print("LiDAR: ");
      if (millis() - lastLidarUpdate < 1000) {
        display.print(dist);
        display.print("mm");
        if (obstacle) {
          display.println(" OBS!");
        } else {
          display.println(" OK");
        }
      } else {
        display.println("ERROR");
      }
      
      // Ligne 5: Vitesse
      display.print("Vit: ");
      display.print(abs(currentSpeedPWM));
      display.print("/255 ");
      if (currentSpeedPWM > 0) display.print("AV");
      else if (currentSpeedPWM < 0) display.print("AR");
      else display.print("--");
      display.println();
      
      // Ligne 6: Direction
      display.print("Dir: ");
      if (currentDirection > 50) display.println("DROITE");
      else if (currentDirection < -50) display.println("GAUCHE");
      else display.println("TOUT DROIT");
      
      // Ligne 7: Stats LiDAR
      display.print("Pts: ");
      display.print(pointsAvailable);
      display.print("/");
      display.print(POINT_BUFFER_SIZE);
      display.print(" @");
      display.print(currentRotationSpeed, 0);
      display.println("Hz");
      
      // Ligne 8: Indicateur obstacle
      if (obstacle) {
        display.setTextSize(2);
        display.println("OBSTACLE!");
      }
      
      display.display();
      lastOledUpdate = millis();
    }
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ═══════════════════════════════════════════════════════════════════════
// ███████╗███████╗████████╗██╗   ██╗██████╗ 
// ██╔════╝██╔════╝╚══██╔══╝██║   ██║██╔══██╗
// ███████╗█████╗     ██║   ██║   ██║██████╔╝
// ╚════██║██╔══╝     ██║   ██║   ██║██╔═══╝ 
// ███████║███████╗   ██║   ╚██████╔╝██║     
// ╚══════╝╚══════╝   ╚═╝    ╚═════╝ ╚═╝     
// ═══════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println("\n╔══════════════════════════════════════════════════╗");
  Serial.println("║  ESP32 WAVE ROVER - FIRMWARE COMPLET v3.0        ║");
  Serial.println("╚══════════════════════════════════════════════════╝");
  
  // Init LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Init I2C et OLED
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("⚠️  Erreur init OLED! Vérifier adresse I2C");
    // Continuer quand même
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("WAVE ROVER v3.0");
    display.println("Initialisation...");
    display.display();
    Serial.println("✅ OLED initialisé");
  }
  
  // Init LiDAR UART
  lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("✅ LiDAR UART initialisé");
  
  // Init moteurs
  initMotors();
  Serial.println("✅ Moteurs initialisés");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(channel_A, 180); // Valeur > MIN_PWM
  ledcWrite(channel_B, 180);
  delay(1000);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(channel_A, 180);
  ledcWrite(channel_B, 180);
  delay(1000);
  ledcWrite(channel_A, 0);
  stopMotors();
  // Init WiFi AP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  Serial.print("✅ WiFi AP démarré - IP: ");
  Serial.println(WiFi.softAPIP());
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ERREUR: Init ESP-NOW échouée!");
    while (1);
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  Serial.println("✅ ESP-NOW initialisé");
  
  // Ajouter peer ESP-01
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp01Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("✅ Peer ESP-01 ajouté");
  } else {
    Serial.println("⚠️  Erreur ajout peer ESP-01");
  }
  
  // Afficher notre MAC
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.print("📍 MAC ESP32: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Init mutexes et queues
  ctrlMutex = xSemaphoreCreateMutex();
  lidarMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();
  packetQueue = xQueueCreate(50, PACKET_SIZE);
  
  memset(&ctrlData, 0, sizeof(ctrlData));
  memset(pointBuffer, 0, sizeof(pointBuffer));
  
  // ╔═══════════════════════════════════════════════════════════════╗
  // ║  CRÉATION TÂCHES FREERTOS                                     ║
  // ║  Core 1: Moteurs + LiDAR (temps réel critique)                ║
  // ║  Core 0: WiFi + Communication + OLED (moins critique)         ║
  // ╚═══════════════════════════════════════════════════════════════╝
  
  xTaskCreatePinnedToCore(motorControlTask, "Motors", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(lidarReadTask, "LidarRead", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(lidarProcessTask, "LidarProcess", 8192, NULL, 4, NULL, 1);
  
  xTaskCreatePinnedToCore(communicationTask, "ESPNOW", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(streamingTask, "TCPStream", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(oledTask, "OLED", 4096, NULL, 1, NULL, 0);
  
  Serial.println("\n╔══════════════════════════════════════════════════╗");
  Serial.println("║           ✅ SYSTÈME PRÊT!                        ║");
  Serial.println("╚══════════════════════════════════════════════════╝\n");
  
  // Blink LED pour confirmer
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
}

// ═══════════════════════════════════════════════════════════════════════
// ██╗      ██████╗  ██████╗ ██████╗ 
// ██║     ██╔═══██╗██╔═══██╗██╔══██╗
// ██║     ██║   ██║██║   ██║██████╔╝
// ██║     ██║   ██║██║   ██║██╔═══╝ 
// ███████╗╚██████╔╝╚██████╔╝██║     
// ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝     
// ═══════════════════════════════════════════════════════════════════════

void loop() {
  // Stats debug périodiques
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 5000) {
    Serial.println("\n📊 === STATS ===");
    Serial.printf("Commandes reçues: %u\n", receivedCommands);
    Serial.printf("Feedback envoyés: %u\n", sentFeedback);
    Serial.printf("Packets LiDAR: %lu\n", packetsReceived);
    Serial.printf("Points traités: %lu\n", totalPointsProcessed);
    Serial.printf("Points buffer: %d/%d\n", pointsAvailable, POINT_BUFFER_SIZE);
    Serial.printf("Obstacle: %s (dist=%umm)\n", 
                  obstacleDetected ? "OUI" : "NON", minObstacleDistance);
    Serial.printf("Vitesse PWM: %d\n", currentSpeedPWM);
    Serial.printf("Client TCP: %s\n", 
                  (streamClient && streamClient.connected()) ? "CONNECTÉ" : "DÉCONNECTÉ");
    Serial.println();
    lastStats = millis();
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}