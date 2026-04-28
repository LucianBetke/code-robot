// ============================================================
// File: Control.cpp
// ============================================================

#include "Control.h"
#include "control_rear.h"
#include "Hardware.h"

SpeedWeg speed[WHEEL_COUNT] =
{
    SpeedWeg(enc[Li]),
    SpeedWeg(enc[Re])
};

PIRegler regler[WHEEL_COUNT] = {
    PIRegler(PI_PARAMS[Li].Kp, PI_PARAMS[Li].Ki,
             -MAX_PWM, MAX_PWM,
             SLEW_LIMIT_PWM),

    PIRegler(PI_PARAMS[Re].Kp, PI_PARAMS[Re].Ki,
             -MAX_PWM, MAX_PWM,
             SLEW_LIMIT_PWM)
};

Rad rad[WHEEL_COUNT] =
{
    Rad(motor[Li], speed[Li], regler[Li], RAD_REGEL_DT_MS, DEAD_PWM[Li]),
    Rad(motor[Re], speed[Re], regler[Re], RAD_REGEL_DT_MS, DEAD_PWM[Re])
};

void speed_reset_all()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        speed[i].reset();
        speed[i].setTimeoutMs(500);
    }
}

void control_update(uint32_t now)
{
    //Serial.println("control_update");
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        rad[i].update(now);
    }
}

// ---------------------------------
// NEU
// ---------------------------------

void control_setSoll(uint8_t w, float v)
{
    if (w < WHEEL_COUNT)
        rad[w].setSoll(v);
}

void control_stopAll()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
        rad[i].stop();
}