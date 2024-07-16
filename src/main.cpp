#include <Arduino.h>
#include <QTRSensors.h>
#include "motor_speed_headers.h"
#include "pid_headers.h"
#include "sensor_utils/sensor_utils.h"
#include "ultrasonic_sensor/ultrasonic_sensor_headers.h"

QTRSensors qtr;

const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];

const uint8_t SMOOTHING_WINDOW = 5;
int16_t errorBuffer[SMOOTHING_WINDOW];
uint8_t errorIndex = 0;
int16_t lastSmoothedError = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(1000);
    }

    uint8_t sensorPins[SENSOR_COUNT] = {32, 33, 25, 26, 27, 14, 12, 13};

    setupUltrasonicSensors();

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);

    Serial.println("Calibration starting in 10 seconds...");
    for (int i = 0; i < 10; i++) {
        Serial.print(10 - i);
        Serial.println(" seconds remaining to start calibration.");
        delay(1000);
    }
    Serial.println("Calibration started...");
    delay(1000);
    
    const uint16_t calibrationSteps = 250;
    for (uint16_t i = 0; i < calibrationSteps; i++) {
        qtr.calibrate();
        delay(20);
        if (i % 25 == 0) {
            Serial.print("Calibration: ");
            Serial.print((i * 100) / calibrationSteps);
            Serial.println("% completed");
        }
    }

    Serial.println("Calibration completed. Status: OK!");

    for (uint8_t i = 0; i < SMOOTHING_WINDOW; i++) {
        errorBuffer[i] = 0;
    }
}

void loop() {
    static int16_t lastError = 0;

    int16_t position = qtr.readLineBlack(sensorValues);
    int16_t error = position - 3500;
    error = smoothError(error, errorBuffer, SMOOTHING_WINDOW, &errorIndex);
    int16_t motorSpeed = KP * error + KD * (error - lastSmoothedError);
    lastSmoothedError = error;

    int16_t m1Speed = BASE_SPEED + motorSpeed;
    int16_t m2Speed = BASE_SPEED - motorSpeed;

    m1Speed = constrain(m1Speed, 0, MAX_SPEED);
    m2Speed = constrain(m2Speed, 0, MAX_SPEED);

    moveMotors(m1Speed, m2Speed);

    long distances[4];
    for (int i = 0; i < 4; i++) {
        distances[i] = measureDistance(ultrasonic_sensors[i]);
    }

    delay(400);
}
