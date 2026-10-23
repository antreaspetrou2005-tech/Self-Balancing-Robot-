#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// FINAL TEST: ESP32 + MPU6050 + L298N + TT motors
// Same pinout and PID gains as v1.9.

#define ENA 23
#define IN1 26
#define IN2 25
#define ENB 32
#define IN3 33
#define IN4 14

MPU6050 mpu(Wire);

float Kp = 12.0f;
float Ki = 100.0f;
float Kd = 0.20f;
const float TARGET_ANGLE = -1.7f;

const float ANGLE_DEADBAND = 0.20f;
const float MAX_ANGLE = 30.0f;
const float FILTER_TAU = 0.10f;
const unsigned long LOOP_US = 10000;
const int MOTOR_DIRECTION = 1;

// Motor behavior: starting requires more PWM than keeping a motor moving.
// Both wheels receive the same PWM because there are no encoders.
const int START_PWM = 85;
const int RUN_PWM = 72;
const int MAX_PWM = 255;
const float START_COMMAND = 1.5f;
const float STOP_COMMAND = 0.5f;
const float FULL_PWM_COMMAND = 150.0f;
const int PWM_SLEW_PER_LOOP = 20;  // maximum increase/decrease each 10 ms

float angle = 0.0f;
float gyroRate = 0.0f;
float integral = 0.0f;
float filteredGyroRate = 0.0f;
unsigned long lastLoop = 0;
unsigned long lastPrint = 0;
bool safetyStopped = false;

bool motorsActive = false;
int appliedPwm = 0;
int appliedDirection = 0;

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  motorsActive = false;
  appliedPwm = 0;
  appliedDirection = 0;
}

void setDirection(int direction) {
  if (direction > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void driveMotors(float command) {
  float magnitude = fabsf(command);
  int direction = (command >= 0.0f ? 1 : -1) * MOTOR_DIRECTION;

  if (!motorsActive) {
    if (magnitude < START_COMMAND) {
      stopMotors();
      return;
    }
    motorsActive = true;
    appliedPwm = 0;
    appliedDirection = direction;
    setDirection(direction);
  } else {
    if (magnitude < STOP_COMMAND) {
      stopMotors();
      return;
    }
    if (direction != appliedDirection) {
      // Direction changes should never happen while applying PWM.
      ledcWrite(ENA, 0);
      ledcWrite(ENB, 0);
      appliedPwm = 0;
      appliedDirection = direction;
      setDirection(direction);
    }
  }

  int basePwm = (appliedPwm == 0) ? START_PWM : RUN_PWM;
  int requestedPwm = basePwm + (int)((MAX_PWM - basePwm) *
                              constrain(magnitude, 0.0f, FULL_PWM_COMMAND) /
                              FULL_PWM_COMMAND);
  requestedPwm = constrain(requestedPwm, START_PWM, MAX_PWM);

  // Limit changes near upright, but allow immediate full power during a fall.
  if (magnitude < 60.0f && appliedPwm > 0) {
    requestedPwm = constrain(requestedPwm,
                             max(0, appliedPwm - PWM_SLEW_PER_LOOP),
                             min(MAX_PWM, appliedPwm + PWM_SLEW_PER_LOOP));
  }
  appliedPwm = requestedPwm;
  ledcWrite(ENA, appliedPwm);
  ledcWrite(ENB, appliedPwm);
}

float getAccelAngle() {
  return atan2f(-mpu.getAccX(), -mpu.getAccZ()) * 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);
  stopMotors();

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("Starting MPU6050...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 ERROR");
    while (true) { stopMotors(); delay(100); }
  }

  Serial.println("Keep robot UPRIGHT and STILL");
  delay(1500);
  mpu.calcOffsets(true, false);
  mpu.update();
  angle = getAccelAngle();
  integral = 0.0f;
  filteredGyroRate = -mpu.getGyroY();
  lastLoop = micros();
  Serial.println("READY - FINAL TEST");
}

void loop() {
  unsigned long now = micros();
  if ((unsigned long)(now - lastLoop) < LOOP_US) return;
  float dt = (now - lastLoop) / 1000000.0f;
  lastLoop = now;
  mpu.update();
  if (dt <= 0.0f || dt > 0.1f) {
    stopMotors();
    integral = 0.0f;
    return;
  }

  float accelAngle = getAccelAngle();
  gyroRate = -mpu.getGyroY();
  float alpha = FILTER_TAU / (FILTER_TAU + dt);
  float predictedAngle = angle + gyroRate * dt;
  float difference = accelAngle - predictedAngle;
  if (fabsf(difference) < 8.0f) {
    angle = alpha * predictedAngle + (1.0f - alpha) * accelAngle;
  } else {
    angle = predictedAngle;
  }

  if (fabsf(angle) > MAX_ANGLE) {
    safetyStopped = true;
    stopMotors();
    integral = 0.0f;
  }
  if (safetyStopped) {
    if (fabsf(angle) < 5.0f && fabsf(gyroRate) < 15.0f) {
      safetyStopped = false;
      integral = 0.0f;
    } else {
      stopMotors();
      return;
    }
  }

  float error = angle - TARGET_ANGLE;
  if (fabsf(error) < ANGLE_DEADBAND) error = 0.0f;

  if (fabsf(error) < 3.0f) {
    integral += error * dt;
    if (fabsf(error) < 0.5f) integral *= expf(-dt / 1.0f);
  } else {
    integral = 0.0f;
  }
  integral = constrain(integral, -0.20f, 0.20f);

  // D filtering: keep it gentle and consistent with the previous stable setup.
  float dAlpha = dt / (0.02f + dt);
  filteredGyroRate += dAlpha * (gyroRate - filteredGyroRate);

  float control = Kp * error + Ki * integral + Kd * filteredGyroRate;
  control = constrain(control, -255.0f, 255.0f);
  driveMotors(control);

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print("ANGLE: "); Serial.print(angle, 2);
    Serial.print(" | GYRO: "); Serial.print(gyroRate, 2);
    Serial.print(" | Error: "); Serial.print(error, 2);
    Serial.print(" | Integral: "); Serial.print(integral, 2);
    Serial.print(" | CONTROL: "); Serial.print(control, 1);
    Serial.print(" | PWM: "); Serial.println(appliedPwm);
  }
}
