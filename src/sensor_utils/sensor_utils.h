#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include <Arduino.h>
#include "ultrasonic_sensor_headers.h"

int16_t smoothError(int16_t currentError, int16_t *buffer, uint8_t windowSize, uint8_t *index);

#endif // SENSOR_UTILS_H
