#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

void moveMotors(int16_t leftSpeed, int16_t rightSpeed);
void printMotorDirection(int16_t leftSpeed, int16_t rightSpeed);

#endif // MOTOR_CONTROL_H
