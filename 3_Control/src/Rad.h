// Rad.h
#pragma once
#include <Arduino.h>
#include "src/ControlTypes.h"
#include "src/Motor.h"
#include "SpeedWeg.h"
#include "PIRegler.h"

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

    int16_t deadPwm() const { return _deadPwm; }
    void setDeadPwm(int16_t deadPwm);
    long countsTotal() const;

private:
    Motor& _motor;
    SpeedWeg& _speed;
    PIRegler& _regler;

    uint16_t _dtMs;
    uint32_t _lastUpdateMs;
    int16_t _lastPwm;
    int16_t _deadPwm;
};