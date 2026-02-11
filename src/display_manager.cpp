#include "display_manager.h"
#include "config.h"
#include "globals.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initDisplay() {
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("WAVE ROVER v3.0");
    display.println("Initializing...");
    display.display();
    Serial.println("OLED initialized");
  }
}

void oledTask(void *pvParameters) {
  while (true) {
    if (millis() - lastOledUpdate > 200) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      
      display.println("=== WAVE ROVER ===");
      
      unsigned long cmdAge = (millis() - lastCommandTime) / 1000;
      display.print("Cmd: ");
      if (cmdAge < 2) {
        display.print("OK (");
        display.print(receivedCommands);
        display.println(")");
      } else {
        display.print("TIMEOUT (");
        display.print(cmdAge);
        display.println("s)");
      }
      
      display.print("WiFi: ");
      display.print(WiFi.RSSI());
      display.print("dBm");
      display.println();
      
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
        display.println(obstacle ? " OBS!" : " OK");
      } else {
        display.println("ERROR");
      }
      
      display.print("Speed: ");
      display.print(abs(currentSpeedPWM));
      display.print("/255 ");
      if (currentSpeedPWM > 0) display.println("FWD");
      else if (currentSpeedPWM < 0) display.println("BWD");
      else display.println("STOP");
      
      display.print("Points: ");
      display.print(pointsAvailable);
      display.print("/");
      display.println(POINT_BUFFER_SIZE);
      
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