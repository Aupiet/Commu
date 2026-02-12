#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// MAC Addresses
extern uint8_t esp01Address[6];

// WiFi AP
#define WIFI_SSID "PorcheWifi"
#define WIFI_PASSWORD "12345678"

// LiDAR Pins & Settings
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 18
#define LIDAR_BAUD 230400

// OLED Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// --- CORRECTION I2C PINS (Wave Rover Standard) ---
#define I2C_SDA 32
#define I2C_SCL 33

// Other Pins
#define LED_PIN 2
#define STREAM_PORT 8888

// LiDAR Parameters
#define OBSTACLE_DISTANCE 300
#define FRONT_ANGLE_MIN 330
#define FRONT_ANGLE_MAX 30
#define PACKET_SIZE 47
#define MEAS_PER_PACKET 12
#define POINT_BUFFER_SIZE 720

// Motor Pins (Ces pins 21/22 ne doivent PAS être utilisés par Wire.begin par
// défaut)
#define PWMA 25
#define AIN2 17
#define AIN1 21
#define BIN1 22
#define BIN2 23
#define PWMB 26

// PWM Settings
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define MOTOR_CHANNEL_A 5
#define MOTOR_CHANNEL_B 6

// Mode de navigation
enum NavMode {
  NAV_NAIVE, // Navigation naïve (obstacle avoidance)
  NAV_SLAM   // Navigation SLAM + A* (futur)
};

// Commande moteur directe (remplace le joystick)
struct MotorCommand {
  int leftPWM;  // -255 à +255
  int rightPWM; // -255 à +255
  uint32_t timestamp;
};

// Structures legacy (gardé pour compatibilité ESP-NOW)
struct ControlPacket {
  int joyX;
  int joyY;
  uint8_t button;
  uint32_t timestamp;
};

#pragma pack(push, 1)
typedef struct {
  uint8_t header;
  uint8_t length;
  uint16_t speed;
  uint16_t start_angle;
  uint16_t distances[MEAS_PER_PACKET];
  uint8_t confidences[MEAS_PER_PACKET];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc;
} LD06Packet;
#pragma pack(pop)

typedef struct {
  float angle;
  uint16_t distance;
  uint8_t confidence;
  float x;
  float y;
  unsigned long ts;
} LidarPoint;

#endif