#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// ======================================
// 1. MPU6050 & HARDWARE FILTER
// ======================================

MPU6050 mpu(Wire);
const uint8_t MPU_ADDR = 0x68;


const float GYRO_SIGN = 1.0f;

// ======================================
// 2. L298N PINS & MOTOR CONFIGURATION
// ======================================

const int ENA = 23;
const int IN1 = 26;
const int IN2 = 25;

const int ENB = 32;
const int IN3 = 33;
const int IN4 = 14;


const int MIN_PWM = 80;


const bool ENABLE_MOTORS = true  ;

// ======================================
// 3. CONTROL LOOP (200 Hz)
// ======================================

const uint32_t interval = 5000; // 5000 us = 5 ms (200 Hz)

uint32_t previousTime = 0;
uint32_t previousPrintTime = 0;

// ======================================
// 4. COMPLEMENTARY FILTER
// ======================================

// Σταθερά χρόνου φίλτρου (με dt = 0.005s δίνει alpha ≈ 0.99)
float tau = 0.5f;

float previousAngle = 0.0f;
float filteredAngle = 0.0f;

// ======================================
// 5. PID PARAMETERS
// ======================================

float Kp = 22.0f;
float Ki = 5.0f;
float Kd = 0.6f;

float setpoint = 0.0f;
float integral = 0.0f;

// Όριο γωνίας πτώσης (σε μοίρες) για αυτόματη διακοπή των μοτέρ
const float FALL_ANGLE_LIMIT = 40.0f;

// ======================================
// 6. MOTOR FUNCTIONS
// ======================================

void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void setMotors(float output) {
    // Αν η εντολή είναι σχεδόν μηδενική, σβήνουμε τα μοτέρ
    if (fabsf(output) < 0.5f) {
        stopMotors();
        return;
    }

    output = constrain(output, -255.0f, 255.0f);
    float absOut = fabsf(output);

    // Χαρτογράφηση του εύρους (0..255) στο ενεργό εύρος των μοτέρ (MIN_PWM..255)
    int pwm = (int)roundf(MIN_PWM + (absOut / 255.0f) * (255.0f - MIN_PWM));
    pwm = constrain(pwm, 0, 255);

    if (output > 0.0f) {
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

    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);
}

// ======================================
// 7. SETUP
// ======================================

void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // PWM configuration (ESP32 Arduino Core 3.x)
    analogWriteResolution(ENA, 8);
    analogWriteResolution(ENB, 8);

    analogWriteFrequency(ENA, 1000);
    analogWriteFrequency(ENB, 1000);

    stopMotors();

    // Start I2C στα 400 kHz (Fast Mode) για γρήγορη ανάγνωση
    Wire.begin(21, 22);
    Wire.setClock(400000);

    delay(300);

    byte status = mpu.begin(0, 0);

    if (status != 0) {
        Serial.println("MPU initialization failed");
        while (true) {
            stopMotors();
            delay(1000);
        }
    }

    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); 
    Wire.write(0x04); 
    Wire.endTransmission(true);

    // Calibrate gyro offsets
    Serial.println("Keep the robot stationary!");
    mpu.calcOffsets(true, false);
    Serial.println("Gyro calibration complete");

    Serial.print("Gyro Y bias: ");
    Serial.println(mpu.getGyroYoffset(), 4);

    // Initial MPU measurements
    mpu.update();

    float ax = mpu.getAccX();
    float az = mpu.getAccZ();

    float ACCangleDeg = atan2(ax, -az) * 180.0f / PI;

    previousAngle = ACCangleDeg;
    filteredAngle = ACCangleDeg;

    previousTime = micros();
    previousPrintTime = millis();

    Serial.println("System ready!");
}

// ======================================
// 8. MAIN CONTROL LOOP
// ======================================

void loop() {
    uint32_t currentTime = micros();
    uint32_t elapsed = currentTime - previousTime;

    if (elapsed < interval) {
        return;
    }

    float dt = elapsed / 1000000.0f;
    previousTime = currentTime;

    // Large delay protection: reset timing/filter state
    if (dt > 1.0f) {
        stopMotors();
        integral = 0.0f;

        mpu.update();

        float ax = mpu.getAccX();
        float az = mpu.getAccZ();

        filteredAngle = atan2(ax, -az) * 180.0f / PI;
        previousAngle = filteredAngle;
        previousTime = micros();

        return;
    }

    // -----------------------------------
    // STEP 1: READ MPU6050
    // -----------------------------------

    mpu.update();

    float ax = mpu.getAccX();
    float az = mpu.getAccZ();

    // Εφαρμογή του GYRO_SIGN ώστε η φορά του γυροσκοπίου να συμφωνεί με το atan2
    float gyroRateCorrected = GYRO_SIGN * mpu.getGyroY();

    // -----------------------------------
    // STEP 2: ACCELEROMETER ANGLE
    // -----------------------------------

    float ACCangleDeg = atan2(ax, -az) * 180.0f / PI;

    // -----------------------------------
    // STEP 3: GYROSCOPE ANGLE
    // -----------------------------------

    float GYROangle = previousAngle + gyroRateCorrected * dt;

    // -----------------------------------
    // STEP 4: COMPLEMENTARY FILTER
    // -----------------------------------

    float alpha = tau / (tau + dt);

    filteredAngle = alpha * GYROangle + (1.0f - alpha) * ACCangleDeg;
    previousAngle = filteredAngle;

    // -----------------------------------
    // STEP 5: PID
    // -----------------------------------

    float error = setpoint - filteredAngle;

    integral += error * dt;
    integral = constrain(integral, -50.0f, 50.0f);

    float P = Kp * error;
    float I = Ki * integral;
    float D = -Kd * gyroRateCorrected;

    float output = P + I + D;

    // -----------------------------------
    // STEP 6: OUTPUT LIMIT & FALL CUTOFF
    // -----------------------------------

    output = constrain(output, -255.0f, 255.0f);

    // Αν το ρομπότ γείρει πάνω από τις 40 μοίρες (έπεσε) ή αν τα μοτέρ είναι 
    // απενεργοποιημένα μέσω του ENABLE_MOTORS, σβήνουμε τα μοτέρ και μηδενίζουμε το I.
    if (!ENABLE_MOTORS || fabsf(filteredAngle) > FALL_ANGLE_LIMIT) {
        stopMotors();
        integral = 0.0f;
    } else {
        setMotors(output);
    }

    // -----------------------------------
    // STEP 7: SERIAL MONITOR
    // -----------------------------------

    if (millis() - previousPrintTime >= 100) {
        previousPrintTime = millis();

        Serial.print("Acc Angle: ");
        Serial.print(ACCangleDeg, 2);

        Serial.print(" | Filtered Angle: ");
        Serial.print(filteredAngle, 2);

        Serial.print(" | GyroY: ");
        Serial.print(gyroRateCorrected, 2);

        Serial.print(" | PID Out: ");
        Serial.println(output, 2);
    }
}
