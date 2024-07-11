#include <Arduino.h>
#include <QTRSensors.h>

#include "sensor_headers.h" // Include sensor headers

QTRSensors qtr;

const float KP = 0.1;
const float KD = 5;

const int16_t BASE_SPEED = 100;

const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];


struct Sensor {
    uint8_t triggerPin;
    uint8_t echoPin;
};

Sensor sensors[4] = {
    {TRIGGER_PIN_LEFT, ECHO_PIN_LEFT},
    {TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT},
    {TRIGGER_PIN_FRONT, ECHO_PIN_FRONT},
    {TRIGGER_PIN_REAR, ECHO_PIN_REAR}
};

void setupSensors() {
    for (int i = 0; i < 4; i++) {
        pinMode(sensors[i].triggerPin, OUTPUT);
        pinMode(sensors[i].echoPin, INPUT);
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

void printMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    Serial.print("Sol Motor Hızı: ");
    Serial.print(leftSpeed);
    Serial.print(" | Sağ Motor Hızı: ");
    Serial.println(rightSpeed);
}

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        Serial.println("Trying to connect to Serial Monitor...\n");
        delay(1000);
    }
    Serial.println("Serial Monitor connected\n");
    
    uint8_t sensorPins[SENSOR_COUNT] = {14, 27, 26, 25, 33, 32, 13, 12};

    setupSensors();

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);

    // Kalibrasyon ve ilerleme yüzdesi yazdırma
    const uint16_t calibrationSteps = 250;
    for (uint16_t i = 0; i < calibrationSteps; i++) {
        qtr.calibrate();
        delay(20);
        if (i % 25 == 0) { // Her %10 tamamlandığında yazdır
            Serial.print("Kalibrasyon: ");
            Serial.print((i * 100) / calibrationSteps);
            Serial.println("% tamamlandı");
        }
    }

    Serial.println("Kalibrasyon tamamlandı");
}

void loop() {
    static int16_t lastError = 0;

    int16_t position = qtr.readLineBlack(sensorValues);
    int16_t error = position - 3500;
    int16_t motorSpeed = KP * error + KD * (error - lastError);
    lastError = error;

    int16_t m1Speed = BASE_SPEED + motorSpeed;
    int16_t m2Speed = BASE_SPEED - motorSpeed;

    m1Speed = constrain(m1Speed, 0, MAX_SPEED);
    m2Speed = constrain(m2Speed, 0, MAX_SPEED);

    printMotorSpeeds(m1Speed, m2Speed);

    long distances[4];
    for (int i = 0; i < 4; i++) {
        distances[i] = measureDistance(sensors[i]);
    }

    Serial.print("Mesafe Sol: ");
    Serial.print(distances[0]);
    Serial.print(" cm, Sağ: ");
    Serial.print(distances[1]);
    Serial.print(" cm, Ön: ");
    Serial.print(distances[2]);
    Serial.print(" cm, Arka: ");
    Serial.print(distances[3]);
    Serial.println(" cm");

    delay(1000);
}
