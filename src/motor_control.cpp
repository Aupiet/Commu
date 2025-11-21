#include "motor_control.h"
#include "config.h"
#include "globals.h"

const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS) - 1;
const uint16_t MIN_PWM = MAX_PWM / 5;

void initMotors() {
    // Réinitialiser tous les pins moteur
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(MOTOR_CHANNEL_A, 0);
  ledcWrite(MOTOR_CHANNEL_B, 0);
  
  delay(100); // Petite pause pour stabilisation

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  ledcSetup(MOTOR_CHANNEL_A, PWM_FREQ, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMA, MOTOR_CHANNEL_A);
  ledcSetup(MOTOR_CHANNEL_B, PWM_FREQ, ANALOG_WRITE_BITS);
  ledcAttachPin(PWMB, MOTOR_CHANNEL_B);
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  
  Serial.println("Motors initialized");
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(MOTOR_CHANNEL_A, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(MOTOR_CHANNEL_B, 0);
}

void testMotors() {

    channelACtrl(-180);
    channelBCtrl(-180);
    delay(1000);
    channelACtrl(180);
    channelBCtrl(180);
    delay(1000);
    stopMotors();
}

void channelACtrl(float pwmInput) {
  int pwmInt = round(pwmInput);
  if (pwmInt == 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_A, 0);
    return;
  }
  
  if (pwmInt > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(MOTOR_CHANNEL_A, abs(constrain(pwmInt, MIN_PWM, MAX_PWM)));
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_A, abs(constrain(pwmInt, -MAX_PWM, -MIN_PWM)));
  }
}

void channelBCtrl(float pwmInput) {
  int pwmInt = round(pwmInput);
  if (pwmInt == 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_B, 0);
    return;
  }
  
  if (pwmInt > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(MOTOR_CHANNEL_B, abs(constrain(pwmInt, MIN_PWM, MAX_PWM)));
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_B, abs(constrain(pwmInt, -MAX_PWM, -MIN_PWM)));
  }
}

void setMotorSpeedFromJoystick(int joyX, int joyY) {
  //  testMotors();
    
  float forward = (float)joyX / 512.0f;
  float turn = (float)joyY / 512.0f;
  
  float left = forward + turn;
  float right = forward - turn;
  
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);
  
  const int MIN_START_PWM = 70;
  const int MAX_MOTOR_PWM = 255;
  
  auto pwmValue = [=](float val) -> int {
    if (val == 0) return 0;
    int pwm = round(abs(val) * (MAX_MOTOR_PWM - MIN_START_PWM) + MIN_START_PWM);
    return (val > 0) ? pwm : -pwm;
  };
  
  int pwmLeft = pwmValue(left);
  int pwmRight = pwmValue(right);
  
  channelACtrl(pwmRight);
  channelBCtrl(pwmLeft);
}

void motorControlTask(void *pvParameters) {
  while (true) {
    int joyX = 0, joyY = 0;
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      joyX = ctrlData.joyX;
      joyY = ctrlData.joyY;
      xSemaphoreGive(ctrlMutex);
    }
    if (obstacleDetected) {
      stopMotors();
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    setMotorSpeedFromJoystick(joyX, joyY);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}