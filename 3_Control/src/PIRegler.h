// ============================================================
// File: PIRegler.h
// Beschreibung:
//  - Bidirektionaler PI-Regler fuer Radgeschwindigkeit
//  - Geschwindigkeitseinheit: cm/s
//  - Ausgang: signed PWM [-uMax .. +uMax]
//  - Anti-Windup (Clamping)
//  - Slew-Rate Begrenzung
// ============================================================

#pragma once

#include <Arduino.h>

class PIRegler
{
public:
    PIRegler(float Kp, float Ki,
        int16_t uMin, int16_t uMax,
        int16_t slewLimit);

    int16_t update(float v_ist_cms, uint16_t dt_ms);

    void presetOutput(float u0_pwm);

    void setSoll(float v_soll_cms);
    float soll() const;

    float Kp() const { return _Kp; }
    float Ki() const { return _Ki; }

    void reset();

    void setParams(float Kp, float Ki);

private:
    int16_t applySlew(int16_t pwm);

    float   _Kp;
    float   _Ki;

    int16_t _uMin;
    int16_t _uMax;
    int16_t _slewLimit;

    float   _v_soll;
    float   _integral;

    int16_t _uPrev;
};