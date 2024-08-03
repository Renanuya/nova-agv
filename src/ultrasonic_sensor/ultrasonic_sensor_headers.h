#ifndef ULTRASONIC_SENSOR_HEADERS_H
#define ULTRASONIC_SENSOR_HEADERS_H

#include <Arduino.h>

#define TRIGGER_PIN_LEFT_1 18 // Trigger pin for left sensor
#define ECHO_PIN_LEFT_1 35 // Echo pin for left sensor

//! @brief These pins are not correct. FIX IT! Before using this code.
#define TRIGGER_PIN_LEFT_2 0 // Trigger pin for left sensor
#define ECHO_PIN_LEFT_2 0 // Echo pin for left sensor
//! End of the @brief comment.

#define TRIGGER_PIN_RIGHT_1 4 // Trigger pin for right sensor
#define ECHO_PIN_RIGHT_1 34 // Echo pin for right sensor

//! @brief These pins are not correct. FIX IT! Before using this code.
#define TRIGGER_PIN_RIGHT_2 0 // Trigger pin for right sensor
#define ECHO_PIN_RIGHT_2 0 // Echo pin for right sensor
//! End of the @brief comment.

#define TRIGGER_PIN_FRONT_1 0 // Trigger pin for front sensor
#define ECHO_PIN_FRONT_1 39 // Echo pin for front sensor

//! @brief These pins are not correct. FIX IT! Before using this code.
#define TRIGGER_PIN_FRONT_2 0 // Trigger pin for front sensor
#define ECHO_PIN_FRONT_2 0 // Echo pin for front sensor
//! End of the @brief comment.

#define TRIGGER_PIN_REAR 15 // Trigger pin for rear sensor
#define ECHO_PIN_REAR 36 // Echo pin for rear sensor

struct Sensor {
    uint8_t triggerPin;
    uint8_t echoPin;
};

extern Sensor ultrasonic_sensors[7];

void setupUltrasonicSensors();
long measureDistance(Sensor sensor);

#endif // ULTRASONIC_SENSOR_HEADERS_H
