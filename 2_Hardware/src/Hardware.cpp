#include <Arduino.h>
#include "src/RobotConfig.h"
#include "Encoder.h"
#include "Motor.h"
#include "Hardware.h"
#include "hardware_pins.h"

Enc   enc[WHEEL_COUNT];
Motor motor[WHEEL_COUNT];
// Dieser Pin ist board-abhaengig:
// vorne: Sync-Ausgang fuer VIST-Anforderung
// hinten: STBY-Ausgang fuer beide Motortreiber
static uint8_t s_stby_sync = 0;

void hardware_begin(const HardwarePinSet& pins, bool resetEnc)
{
    s_stby_sync = pins.stby_sync;
    enc[Li].begin(pins.encLiA, pins.encLiB, resetEnc);
    enc[Re].begin(pins.encReA, pins.encReB, resetEnc);
    motor[Li].begin(pins.motorLi1, pins.motorLi2, enc[Li]);
    motor[Re].begin(pins.motorRe1, pins.motorRe2, enc[Re]);
    pinMode(s_stby_sync, OUTPUT);
    motor[Li].init();
    motor[Re].init();
}

void hardware_enableMotors()
{
    digitalWrite(s_stby_sync, HIGH);
}

void hardware_disableMotors()
{
    digitalWrite(s_stby_sync, LOW);
}

void hardware_requestVist()
{
    digitalWrite(s_stby_sync, HIGH);
    digitalWrite(s_stby_sync, LOW);
}