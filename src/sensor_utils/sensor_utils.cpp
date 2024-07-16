#include "sensor_utils.h"

int16_t smoothError(int16_t currentError, int16_t *buffer, uint8_t windowSize, uint8_t *index) {
    buffer[*index] = currentError;
    *index = (*index + 1) % windowSize;

    int32_t sum = 0;
    for (uint8_t i = 0; i < windowSize; i++) {
        sum += buffer[i];
    }
    return sum / windowSize;
}
