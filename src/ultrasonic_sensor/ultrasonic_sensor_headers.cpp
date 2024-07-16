#include "ultrasonic_sensor_headers.h"

Sensor ultrasonic_sensors[4] = {
    {TRIGGER_PIN_LEFT, ECHO_PIN_LEFT},
    {TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT},
    {TRIGGER_PIN_FRONT, ECHO_PIN_FRONT},
    {TRIGGER_PIN_REAR, ECHO_PIN_REAR}
};

void setupUltrasonicSensors() {
    for (int i = 0; i < 4; i++) {
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
