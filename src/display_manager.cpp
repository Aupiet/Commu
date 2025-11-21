#include "display_manager.h"
#include "config.h"
#include "globals.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
// <<<<<<<<<<=== === ===SSD1306: 0x3C=== === ===>>>>>>>>>>
// 0.91inch OLED
bool screenDefaultMode = true;

unsigned long currentTimeMillis = millis();
unsigned long lastTimeMillis = millis();

// default
String screenLine_0;
String screenLine_1;
String screenLine_2;
String screenLine_3;

// custom
String customLine_0;
String customLine_1;
String customLine_2;
String customLine_3;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Mutex global pour accès I2C
SemaphoreHandle_t oledMutex = nullptr;


// init oled ctrl functions.
void init_oled(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
  oledMutex = xSemaphoreCreateMutex(); // Initialise le mutex
}


// Updata all data and flash the screen.
void oled_update() {
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(screenLine_0);
  display.println(screenLine_1);
  display.println(screenLine_2);
  display.println(screenLine_3);

  display.display();
}

// oled ctrl.
void oledCtrl(byte inputLineNum, String inputMegs) {
  screenDefaultMode = false;
  switch (inputLineNum) {
  case 0: customLine_0 = inputMegs;break;
  case 1: customLine_1 = inputMegs;break;
  case 2: customLine_2 = inputMegs;break;
  case 3: customLine_3 = inputMegs;break;
  }
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(customLine_0);
  display.println(customLine_1);
  display.println(customLine_2);
  display.println(customLine_3);

  display.display();
}

void oledTask(void *pvParameters) {
    while (true) {
        if (oledMutex && xSemaphoreTake(oledMutex, 50 / portTICK_PERIOD_MS) == pdTRUE) {
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("WIFI SIGNAL:");
            display.setTextSize(2);
            display.setCursor(0, 16);
            display.print(WiFi.RSSI());
            display.print(" dBm");
            display.display();
            xSemaphoreGive(oledMutex);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
