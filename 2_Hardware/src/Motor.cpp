// Motor.cpp
#include "Motor.h"

// ===== Motor =====
Motor::Motor() : _pin1(0), _pin2(0), _enc(nullptr)
{
}

void Motor::init() {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    bremse(false); // Coast
}

void Motor::begin(uint8_t pin1, uint8_t pin2, Enc& enc)
{
    _pin1 = pin1;
    _pin2 = pin2;
    _enc = &enc;
}

void Motor::vor(uint8_t pwm) { analogWrite(_pin1, pwm); digitalWrite(_pin2, LOW); }
void Motor::rueck(uint8_t pwm) { analogWrite(_pin2, pwm); digitalWrite(_pin1, LOW); }

void Motor::bremse(bool art) {
    if (art) { digitalWrite(_pin1, HIGH); digitalWrite(_pin2, HIGH); }   // aktiv bremsen
    else { digitalWrite(_pin1, LOW);  digitalWrite(_pin2, LOW); }   // auslaufen
}

void Motor::attachEnc(Enc& enc) { _enc = &enc; }
Enc* Motor::enc() const { return _enc; }