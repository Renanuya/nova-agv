#include "ultrasonic_sensor_headers.h"

Sensor ultrasonic_sensors[7] = {
    {TRIGGER_PIN_LEFT_1, ECHO_PIN_LEFT_1},
    {TRIGGER_PIN_LEFT_2, ECHO_PIN_LEFT_2},
    {TRIGGER_PIN_RIGHT_1, ECHO_PIN_RIGHT_1},
    {TRIGGER_PIN_RIGHT_2, ECHO_PIN_RIGHT_2},
    {TRIGGER_PIN_FRONT_1, ECHO_PIN_FRONT_1},
    {TRIGGER_PIN_FRONT_2, ECHO_PIN_FRONT_2},
    {TRIGGER_PIN_REAR, ECHO_PIN_REAR}
};

void setupUltrasonicSensors() {
    for (int i = 0; i < 7; i++) {
        pinMode(ultrasonic_sensors[i].triggerPin, OUTPUT);
        pinMode(ultrasonic_sensors[i].echoPin, INPUT);
    }
}

long measureDistance(Sensor sensor) {
    digitalWrite(sensor.triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.triggerPin, LOW);
    long duration = pulseIn(sensor.echoPin, HIGH);
    return duration * 0.034 / 2;
}
