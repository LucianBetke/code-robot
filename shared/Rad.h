// Rad.h
#pragma once
#include <Arduino.h>
#include "globals.h" 
#include "../hardware/Motor.h"
//#include "Motor.h"
#include "SpeedWeg.h"
#include "PIRegler.h"
#include "ControlParams.h"

class Rad {
public:
    Rad(Motor& motor, SpeedWeg& speed, PIRegler& regler, uint16_t dtMs, Wheel radIndex);

    void  setSoll(float v_soll);
    float soll() const;

    void update(uint32_t nowMs);

    int16_t lastPwm() const { return _lastPwm; }
   
    void setTrim(int16_t t);
    int16_t trim() const { return _trim; }
    PIParam getPI() const;
    void reset();

private:
    Motor& _motor;
    SpeedWeg& _speed;
    PIRegler& _regler;

    uint16_t _dtMs;
    Wheel _index;
    uint32_t _lastUpdateMs;
    int16_t  _lastPwm;
    int16_t _trim;
};
