#include <Arduino.h>
#include <QTRSensors.h>
# define maxSpeed 255

QTRSensors qtr;

const float KP = 0.1;
const float KD = 5;

const int16_t M1 = 100;
const int16_t M2 = 100;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void printMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    Serial.print("Sol Motor Hızı: ");
    Serial.print(leftSpeed);
    Serial.print(" | Sağ Motor Hızı: ");
    Serial.println(rightSpeed);
}

void setup() {
    Serial.begin(9600);

    uint8_t sensorPins[SensorCount] = {14,27,26,25,33,32,13,18};

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SensorCount);

    for (uint16_t i = 0; i < 250; i++) 
    {
        qtr.calibrate();
        delay(20);
    }

    Serial.println("Kalibrasyon tamamlandı");
}

void loop() {
    static int16_t lastError = 0;

    int16_t position = qtr.readLineBlack(sensorValues);

    int16_t error = position - 3500;
    
    int16_t motorSpeed = KP * error + KD * (error - lastError);
    lastError = error;

    int16_t m1Speed = M1 + motorSpeed;
    int16_t m2Speed = M2 - motorSpeed;

    if (m1Speed < 0) { m1Speed = 0; }
    if (m2Speed < 0) { m2Speed = 0; }

    if (m1Speed > maxSpeed) { m1Speed = maxSpeed; }
    if (m2Speed > maxSpeed) { m2Speed = maxSpeed; }

    printMotorSpeeds(m1Speed, m2Speed);
    delay(250);
}

