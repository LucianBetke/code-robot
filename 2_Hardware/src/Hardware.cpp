// Hardware.cpp
#include <Arduino.h>
#include "src/globals.h"
#include "Encoder.h"
#include "Motor.h"
#include "Hardware.h"
#include "hardware_pins.h"

Enc   enc[WHEEL_COUNT];
Motor motor[WHEEL_COUNT];

static uint8_t s_stby_pin = 0;

void hardware_begin(const HardwarePinSet& pins, bool resetEnc)
{
    s_stby_pin = pins.stby;

    enc[Li].begin(pins.encLiA, pins.encLiB, resetEnc);
    enc[Re].begin(pins.encReA, pins.encReB, resetEnc);

    motor[Li].begin(pins.motorLi1, pins.motorLi2, enc[Li]);
    motor[Re].begin(pins.motorRe1, pins.motorRe2, enc[Re]);

    pinMode(s_stby_pin, OUTPUT);
    digitalWrite(s_stby_pin, LOW);

    motor[Li].init();
    motor[Re].init();
}

void hardware_enableMotors()
{
    digitalWrite(s_stby_pin, HIGH);
}

void hardware_disableMotors()
{
    digitalWrite(s_stby_pin, LOW);
}
