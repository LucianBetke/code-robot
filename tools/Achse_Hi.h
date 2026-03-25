// Achse_Hi.h
#pragma once
#include <Arduino.h>
#include "Motor.h"
#include "Encoder.h"

class Achse_Hi {
public:
    Achse_Hi(Motor& li, Motor& re);

    void bremse(bool art);

    Motor& hiLi() { return _li; }
    Motor& hiRe() { return _re; }

private:
    Motor& _li;
    Motor& _re;
};
