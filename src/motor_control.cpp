#include "motor_control.h"
#include "config.h"
#include "globals.h"

const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = 255;
const uint16_t MIN_PWM =
    60; // Augmenté légèrement pour éviter le grognement moteur

void initMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

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

  Serial.println("Motors initialized");
}

void stopMotors() {
  // Freinage actif (Low/Low coupe l'alimentation, mais ne freine pas fort.
  // High/High freinerait)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(MOTOR_CHANNEL_A, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(MOTOR_CHANNEL_B, 0);
}

void channelACtrl(int pwmInput) {
  if (pwmInput == 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_A, 0);
    return;
  }
  if (pwmInput > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(MOTOR_CHANNEL_A, abs(pwmInput));
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_A, abs(pwmInput));
  }
}

void channelBCtrl(int pwmInput) {
  if (pwmInput == 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_B, 0);
    return;
  }
  if (pwmInput > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(MOTOR_CHANNEL_B, abs(pwmInput));
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(MOTOR_CHANNEL_B, abs(pwmInput));
  }
}

void setMotorSpeedFromJoystick(int joyX, int joyY) {
  // Normalisation -1.0 à 1.0
  float forward = (float)joyY / 512.0f; // joyY est AVANCER/RECULER
  float turn = (float)joyX / 512.0f;    // joyX est GAUCHE/DROITE

  // Mixage Arcade
  float left = forward + turn;
  float right = forward - turn;

  // Contrainte
  left = constrain(left, -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  // Mapping PWM avec zone morte minimale pour démarrage moteur
  auto pwmMap = [](float val) -> int {
    if (abs(val) < 0.1)
      return 0;
    int pwm = round(abs(val) * (MAX_PWM - MIN_PWM) + MIN_PWM);
    return (val > 0) ? pwm : -pwm;
  };

  int pwmLeft = pwmMap(left);
  int pwmRight = pwmMap(right);

  channelBCtrl(pwmLeft);  // Canal B = Gauche sur Wave Rover
  channelACtrl(pwmRight); // Canal A = Droite sur Wave Rover

  currentSpeedPWM = (pwmLeft + pwmRight) / 2;
}

void motorControlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz control loop

  while (true) {
    int leftPWM = 0, rightPWM = 0;

    // Récupération atomique des commandes moteur
    if (xSemaphoreTake(ctrlMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      leftPWM = motorCmd.leftPWM;
      rightPWM = motorCmd.rightPWM;
      xSemaphoreGive(ctrlMutex);
    }

    // --- SÉCURITÉ LIDAR (dernier rempart) ---
    bool isMovingForward = (leftPWM > 0 && rightPWM > 0);
    bool safeToMove = true;

    if (obstacleDetected && isMovingForward) {
      safeToMove = false;
    }

    if (!safeToMove) {
      stopMotors();
      currentSpeedPWM = 0;
    } else {
      // Appliquer directement les PWM gauche/droite
      channelBCtrl(leftPWM);  // Canal B = Gauche sur Wave Rover
      channelACtrl(rightPWM); // Canal A = Droite sur Wave Rover
      currentSpeedPWM = (leftPWM + rightPWM) / 2;
    }

    // Boucle régulière
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}