#include "motor_control.h"
#include "motor_speed_headers.h"

void moveMotors(int16_t leftSpeed, int16_t rightSpeed) {
    printMotorDirection(leftSpeed, rightSpeed);
}

void printMotorDirection(int16_t leftSpeed, int16_t rightSpeed) {
    Serial.print("L: ");
    Serial.print(leftSpeed);
    Serial.print("  R: ");
    Serial.print(rightSpeed);
    if (leftSpeed == 0 && rightSpeed == 0)
    {
        Serial.print("  STOP");
    }
    else if (leftSpeed > rightSpeed)
    {
        Serial.print("  LEFT");
    }
    else if (leftSpeed < rightSpeed )
    {
        Serial.print("  RIGHT");
    }
    Serial.println();
}
