#ifndef ULTRASONIC_SENSOR_HEADERS_H
#define ULTRASONIC_SENSOR_HEADERS_H

#include <Arduino.h>

#define TRIGGER_PIN_LEFT 18 // Trigger pin for left sensor
#define ECHO_PIN_LEFT 35 // Echo pin for left sensor

#define TRIGGER_PIN_RIGHT 4 // Trigger pin for right sensor
#define ECHO_PIN_RIGHT 34 // Echo pin for right sensor

#define TRIGGER_PIN_FRONT 0 // Trigger pin for front sensor
#define ECHO_PIN_FRONT 39 // Echo pin for front sensor

#define TRIGGER_PIN_REAR 15 // Trigger pin for rear sensor
#define ECHO_PIN_REAR 36 // Echo pin for rear sensor

struct Sensor {
    uint8_t triggerPin;
    uint8_t echoPin;
};

extern Sensor ultrasonic_sensors[4];

void setupUltrasonicSensors();
long measureDistance(Sensor sensor);

#endif // ULTRASONIC_SENSOR_HEADERS_H
