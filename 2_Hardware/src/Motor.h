// ============================================================
// File: Motor.h
// Zweck:
//  - Steuerung eines einzelnen Motors im 2-Pin-Modus
//  - Richtung, PWM-Leistung, Bremse
//  - Optionale Encoder-Anbindung
// ============================================================

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Enc;

class Motor
{
public:
    Motor();

    void begin(uint8_t pin1, uint8_t pin2, Enc& enc);

    void init();

    void vor(uint8_t pwm);
    void rueck(uint8_t pwm);

    void bremse(bool art);

    void attachEnc(Enc& enc);
    Enc* enc() const;

private:
    uint8_t _pin1;
    uint8_t _pin2;

    Enc* _enc;
};

#endif // MOTOR_H