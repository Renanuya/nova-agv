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

bool obstacleDetected = false;
bool firstTurn = true; // Keeps track of the initial turn direction

// Stepper motor control pins
const int stepPinLeft = 8;   // Connect to PUL+ on the CWD860 for left motor
const int dirPinLeft = 9;    // Connect to DIR+ on the CWD860 for left motor
const int enablePinLeft = 10; // Connect to ENA+ on the CWD860 for left motor

const int stepPinRight = 11;  // Connect to PUL+ on the CWD860 for right motor
const int dirPinRight = 12;   // Connect to DIR+ on the CWD860 for right motor
const int enablePinRight = 13; // Connect to ENA+ on the CWD860 for right motor

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(1000);
    }

    uint8_t sensorPins[SENSOR_COUNT] = {32, 33, 25, 26, 27, 14, 12, 13};

    setupUltrasonicSensors();

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);

    // Stepper motor pin setup
    pinMode(stepPinLeft, OUTPUT);
    pinMode(dirPinLeft, OUTPUT);
    pinMode(enablePinLeft, OUTPUT);
    digitalWrite(enablePinLeft, LOW); // Enable the driver for the left motor

    pinMode(stepPinRight, OUTPUT);
    pinMode(dirPinRight, OUTPUT);
    pinMode(enablePinRight, OUTPUT);
    digitalWrite(enablePinRight, LOW); // Enable the driver for the right motor

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

    long distances[7];
    for (int i = 0; i < 7; i++) {
        distances[i] = measureDistance(ultrasonic_sensors[i]);
    }

    // Obstacle detection and avoidance logic
    if (distances[4] <= 80 || distances[5] <= 80) {
        obstacleDetected = true;
        moveMotors(0, 0); // Stop the motors
        delay(5000); // Wait for 5 seconds

        if (distances[4] <= 80 || distances[5] <= 80) {
            if (firstTurn) {
                if (distances[2] > distances[3]) {
                    turnRight();
                    firstTurn = false;
                } else {
                    turnLeft();
                    firstTurn = true;
                }
            } else {
                if (distances[0] > distances[1]) {
                    turnLeft();
                    firstTurn = false;
                } else {
                    turnRight();
                    firstTurn = true;
                }
            }

            delay(2000); // Wait for 2 seconds after turning

            moveForwardForDistance(80); // Move forward for 80 cm

            if (firstTurn) {
                turnOpposite(firstTurn); // Turn to the opposite direction
                delay(2000); // Wait for 2 seconds after turning
                moveForwardForDistance(80); // Move forward for 80 cm
            }

            turnOpposite(firstTurn); // Turn to the original direction
            delay(2000); // Wait for 2 seconds after turning
        }
    } else {
        obstacleDetected = false;
    }

    delay(400);
}

void stepMotor(int steps, bool dir, int stepPin, int dirPin) {
    digitalWrite(dirPin, dir);
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500); // Adjust the delay to control the speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500); // Adjust the delay to control the speed
    }
}

void moveMotors(int m1Speed, int m2Speed) {
    int stepsLeft = abs(m1Speed);
    int stepsRight = abs(m2Speed);
    bool dirLeft = m1Speed > 0;
    bool dirRight = m2Speed > 0;
    int maxSteps = max(stepsLeft, stepsRight);

    for (int i = 0; i < maxSteps; i++) {
        if (i < stepsLeft) {
            stepMotor(1, dirLeft, stepPinLeft, dirPinLeft);
        }
        if (i < stepsRight) {
            stepMotor(1, dirRight, stepPinRight, dirPinRight);
        }
    }
}

void moveForwardForDistance(int cm) {
    int steps = calculateStepsForDistance(cm);
    stepMotor(steps, HIGH, stepPinLeft, dirPinLeft);
    stepMotor(steps, HIGH, stepPinRight, dirPinRight);
}

void turnRight() {
    int steps = calculateStepsForTurn(90); // Adjust this for a 90-degree turn
    stepMotor(steps, HIGH, stepPinLeft, dirPinLeft);
    stepMotor(steps, LOW, stepPinRight, dirPinRight);
}

void turnLeft() {
    int steps = calculateStepsForTurn(90); // Adjust this for a 90-degree turn
    stepMotor(steps, LOW, stepPinLeft, dirPinLeft);
    stepMotor(steps, HIGH, stepPinRight, dirPinRight);
}

void turnOpposite(bool firstTurn) {
    if (firstTurn) {
        turnLeft();
    } else {
        turnRight();
    }
}

int calculateStepsForDistance(int cm) {
    // Implement your method to convert cm to steps
    // This is a placeholder function
    return cm * 10; // Example conversion factor
}

int calculateStepsForTurn(int degrees) {
    // Implement your method to convert degrees to steps
    // This is a placeholder function
    return degrees * 10; // Example conversion factor
}
