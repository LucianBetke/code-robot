// Rad.h
#pragma once
#include <Arduino.h>
#include "globals.h"
#include "Motor.h"
#include "SpeedWeg.h"
#include "PIRegler.h"
#include "ControlParams.h"

class Rad {
public:
    Rad(Motor& motor, SpeedWeg& speed, PIRegler& regler,
        uint16_t dtMs, int16_t deadPwm);

    void  setSoll(float v_soll);
    float soll() const;
    float vIst() const;

    void update(uint32_t nowMs);

    int16_t lastPwm() const { return _lastPwm; }

    PIParam getPI() const;
    void reset();
    void stop();

private:
    Motor& _motor;
    SpeedWeg& _speed;
    PIRegler& _regler;

    uint16_t _dtMs;
    uint32_t _lastUpdateMs;
    int16_t _lastPwm;
    int16_t _deadPwm;   // neu
};