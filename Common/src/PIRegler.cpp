// PIRegler.cpp  (bidirektional)

#include "PIRegler.h"
#include <Arduino.h>
#include <math.h> // fabsf

PIRegler::PIRegler(float Kp, float Ki,
    int16_t uMin, int16_t uMax,
    int16_t slewLimit)
    : _Kp(Kp), _Ki(Ki),
    _uMin(uMin), _uMax(uMax),
    _slewLimit(slewLimit),
    _v_soll(0.0f),
    _integral(0.0f),
    _uPrev(0),
    _lastE(0.0f),
    _lastP(0.0f),
    _lastI(0.0f)
{
}

int16_t PIRegler::update(float v_ist, uint16_t dt_ms) {
    if (dt_ms == 0) { return _uPrev; }

    // --- Fehler ---
    const float e = _v_soll - v_ist;
    _lastE = e;

    // --- Zeit ---
    const float dt_s = (float)dt_ms * 0.001f;

    // --- bidirektional: u_norm in [-1..+1] ---
    const float uMinNorm = -1.0f;
    const float uMaxNorm = +1.0f;

    // --- PI vor Begrenzung ---
    const float p_part = _Kp * e;
    const float u_pre = p_part + _integral;

    // --- Begrenzen (Sättigung) ---
    const float u_limited = constrain(u_pre, uMinNorm, uMaxNorm);

    // --- Anti-Windup: Clamping (Integrator nur, wenn nicht gesättigt) ---
    const bool saturated = (u_pre <= uMinNorm) || (u_pre >= uMaxNorm);
    if (!saturated) {
        _integral += _Ki * e * dt_s;
    }

    // --- endgültiger PI-Ausgang ---
    float u = p_part + _integral;
    u = constrain(u, uMinNorm, uMaxNorm);

    _lastP = p_part;
    _lastI = _integral;

    // --- PWM signed in [-255..+255] ---
    // korrekt runden mit Vorzeichen:
    float pwm_f = u * 255.0f;
    if (pwm_f >= 0.0f) pwm_f += 0.5f;
    else               pwm_f -= 0.5f;

    int16_t pwm = (int16_t)pwm_f;

    // --- Grenzen: signed ---
    // Interpretation:
    //  - _uMax ist der positive Maximalwert
    //  - _uMin kann negativ sein (z.B. -MAX_PWM) oder 0 (wenn du ihn nicht nutzt)
    // Sicherer Standard: [-_uMax..+_uMax], aber respektiere _uMin, falls negativ gesetzt.
    int16_t pwmMax = _uMax;
    int16_t pwmMin = _uMin;

    if (pwmMax < 0) pwmMax = (int16_t)(-pwmMax); // defensiv
    if (pwmMin > 0) pwmMin = (int16_t)(-pwmMax); // wenn jemand uMin falsch positiv setzt, mach symmetrisch

    // Wenn uMin nicht negativ gesetzt ist, mache symmetrisch:
    if (pwmMin >= 0) pwmMin = (int16_t)(-pwmMax);

    pwm = constrain(pwm, pwmMin, pwmMax);

    // --- Slewrate ---
    pwm = applySlew(pwm);

    return pwm;
}

int16_t PIRegler::applySlew(int16_t pwm)
{
    const int16_t du = pwm - _uPrev;

    if (du > _slewLimit)        pwm = _uPrev + _slewLimit;
    else if (du < -_slewLimit)  pwm = _uPrev - _slewLimit;

    _uPrev = pwm;
    return pwm;
}

void PIRegler::presetOutput(float u0_pwm) {
    // bidirektional clamp auf [-uMax..+uMax] (oder uMin, wenn negativ gesetzt)
    int16_t pwmMax = _uMax;
    int16_t pwmMin = _uMin;

    if (pwmMax < 0) pwmMax = (int16_t)(-pwmMax);
    if (pwmMin >= 0) pwmMin = (int16_t)(-pwmMax);

    if (u0_pwm > (float)pwmMax) u0_pwm = (float)pwmMax;
    if (u0_pwm < (float)pwmMin) u0_pwm = (float)pwmMin;

    // Normierung auf [-1..+1]
    const float u0_norm = u0_pwm / 255.0f;

    // Integrator-Vorbelegung (Arbeitspunkt)
    _integral = u0_norm;

    // Slew-Startwert auf diesen Ausgang setzen (signed)
    float pwm_f = u0_pwm;
    if (pwm_f >= 0.0f) pwm_f += 0.5f;
    else               pwm_f -= 0.5f;
    _uPrev = (int16_t)pwm_f;
}

void PIRegler::setSoll(float v_soll) { _v_soll = v_soll; }

float PIRegler::soll() const { return _v_soll; }

void PIRegler::reset() {
    _integral = 0.0f;
    _uPrev = 0;

    _lastE = 0.0f;
    _lastP = 0.0f;
    _lastI = 0.0f;
}
