#include <wiringPi.h>
#include <softPwm.h>

// https://pinout.xyz/pinout/wiringpi
#define MOTOR_1A 1 // GPIO18
#define MOTOR_1B 26 // GPIO12
#define MOTOR_2A 23 // GPIO13
#define MOTOR_2B 24 // GPIO19

int main(void) {
    wiringPiSetup();
    pinMode(MOTOR_1A, OUTPUT);
    pinMode(MOTOR_1B, OUTPUT);
    pinMode(MOTOR_2A, OUTPUT);
    pinMode(MOTOR_2B, OUTPUT);

    softPwmCreate(MOTOR_1B, 0, 100);
    softPwmCreate(MOTOR_2B, 0, 100);

    // stop
    digitalWrite(MOTOR_1A, LOW);
    digitalWrite(MOTOR_1B, LOW);
    digitalWrite(MOTOR_2A, LOW);
    digitalWrite(MOTOR_2B, LOW);

    // left forward
    digitalWrite(MOTOR_1A, LOW);
    softPwmWrite(MOTOR_1B, 100);
//    digitalWrite(MOTOR_1B, HIGH);
    delay(500);
    digitalWrite(MOTOR_1A, LOW);
    softPwmWrite(MOTOR_1B, 0);
    delay(10);

    // right forward
    digitalWrite(MOTOR_2A, LOW);
//    digitalWrite(MOTOR_2B, HIGH);
    softPwmWrite(MOTOR_2B, 100);
    delay(500);
    digitalWrite(MOTOR_2A, LOW);
    softPwmWrite(MOTOR_2B, 0);
    delay(10);

    return 0;
}