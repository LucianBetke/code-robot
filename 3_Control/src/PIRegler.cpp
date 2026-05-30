// ============================================================
// File: PIRegler.cpp
// Beschreibung:
//  - Bidirektionaler PI-Regler für Drehzahlregelung
//  - Ausgang: signed PWM [-uMax .. +uMax]
//  - Anti-Windup (Clamping)
//  - Slew-Rate Begrenzung
// ============================================================

#include "PIRegler.h"
#include <Arduino.h>

static void normalizePwmLimits(int16_t configuredMin, int16_t configuredMax,
    int16_t& pwmMin, int16_t& pwmMax)
{
    if (configuredMin <= configuredMax)
    {
        pwmMin = configuredMin;
        pwmMax = configuredMax;
    }
    else
    {
        pwmMin = configuredMax;
        pwmMax = configuredMin;
    }
}

PIRegler::PIRegler(float Kp, float Ki,
    int16_t uMin, int16_t uMax,
    int16_t slewLimit)
    : _Kp(Kp), _Ki(Ki),
    _uMin(uMin), _uMax(uMax),
    _slewLimit(slewLimit),
    _v_soll(0.0f),
    _integral(0.0f),
    _uPrev(0)
{
}

int16_t PIRegler::update(float v_ist, uint16_t dt_ms)
{
    if (dt_ms == 0)
    {
        return _uPrev;
    }

    // --- Fehler ---
    const float e = _v_soll - v_ist;

    // --- Zeit ---
    const float dt_s = (float)dt_ms * 0.001f;

    // --- bidirektional: u_norm in [-1..+1] ---
    const float uMinNorm = -1.0f;
    const float uMaxNorm = +1.0f;

    // --- PI vor Begrenzung ---
    const float p_part = _Kp * e;
    const float u_pre = p_part + _integral;

    // --- Anti-Windup: Clamping ---
    // Integrator nur weiterführen, solange der unbegrenzte Ausgang
    // noch nicht außerhalb des erlaubten Bereichs liegt.
    const bool saturated = (u_pre <= uMinNorm) || (u_pre >= uMaxNorm);

    if (!saturated)
    {
        _integral += _Ki * e * dt_s;
    }

    // --- endgültiger PI-Ausgang ---
    float u = p_part + _integral;
    u = constrain(u, uMinNorm, uMaxNorm);

    // --- PWM signed in [-255..+255] ---
    float pwm_f = u * 255.0f;

    if (pwm_f >= 0.0f)
    {
        pwm_f += 0.5f;
    }
    else
    {
        pwm_f -= 0.5f;
    }

    int16_t pwm = (int16_t)pwm_f;

    // --- Grenzen: signed, robust gegen vertauschte Parameter ---
    int16_t pwmMin;
    int16_t pwmMax;
    normalizePwmLimits(_uMin, _uMax, pwmMin, pwmMax);

    pwm = constrain(pwm, pwmMin, pwmMax);

    // --- Slew-Rate ---
    pwm = applySlew(pwm);

    return pwm;
}

int16_t PIRegler::applySlew(int16_t pwm)
{
    const int16_t du = pwm - _uPrev;

    if (du > _slewLimit)
    {
        pwm = _uPrev + _slewLimit;
    }
    else if (du < -_slewLimit)
    {
        pwm = _uPrev - _slewLimit;
    }

    _uPrev = pwm;
    return pwm;
}

void PIRegler::presetOutput(float u0_pwm)
{
    // --- Grenzen: signed, robust gegen vertauschte Parameter ---
    int16_t pwmMin;
    int16_t pwmMax;
    normalizePwmLimits(_uMin, _uMax, pwmMin, pwmMax);

    if (u0_pwm > (float)pwmMax)
    {
        u0_pwm = (float)pwmMax;
    }

    if (u0_pwm < (float)pwmMin)
    {
        u0_pwm = (float)pwmMin;
    }

    const float u0_norm = u0_pwm / 255.0f;
    _integral = u0_norm;

    float pwm_f = u0_pwm;

    if (pwm_f >= 0.0f)
    {
        pwm_f += 0.5f;
    }
    else
    {
        pwm_f -= 0.5f;
    }

    _uPrev = (int16_t)pwm_f;
}

void PIRegler::setSoll(float v_soll)
{
    _v_soll = v_soll;
}

float PIRegler::soll() const
{
    return _v_soll;
}

void PIRegler::reset()
{
    _integral = 0.0f;
    _uPrev = 0;
}

void PIRegler::setParams(float Kp, float Ki)
{
    _Kp = Kp;
    _Ki = Ki;
}