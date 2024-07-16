#ifndef MOTOR_SPEED_HEADERS_H
#define MOTOR_SPEED_HEADERS_H

#include <Arduino.h> // Ensure types like int16_t and uint8_t are defined
#pragma once // Ensures this header file is only included once
#include <stdint.h> // Include the standard integer library

#define MAX_SPEED 255 // Maximum speed value for motors
#define MIN_SPEED -255 // Minimum speed value for motors
#define STOP_SPEED 0 // Stop speed value for motors
#define BASE_SPEED 100 // Base speed value for motors

void printMotorDirection(int16_t leftSpeed, int16_t rightSpeed);
void moveMotors(int16_t leftSpeed, int16_t rightSpeed);

#endif // MOTOR_SPEED_HEADERS_H
