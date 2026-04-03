// ============================================================
// File: PIRegler.h
// Beschreibung:
//  - Bidirektionaler PI-Regler für Drehzahlregelung
//  - Ausgang: signed PWM [-uMax .. +uMax]
//  - Anti-Windup (Clamping)
//  - Slew-Rate Begrenzung
// ============================================================

#pragma once
#include <Arduino.h>

class PIRegler {
public:
    PIRegler(float Kp, float Ki,
        int16_t uMin, int16_t uMax,
        int16_t slewLimit);

    // v_ist in m/s, dt_ms = Zeit seit letztem Aufruf
    int16_t update(float v_ist, uint16_t dt_ms);

    // Start-PWM vorgeben (Feedforward / Arbeitspunkt-Vorbelegung)
    void presetOutput(float u0_pwm);

    void setSoll(float v_soll);
    float soll() const;

    float Kp() const { return _Kp; }
    float Ki() const { return _Ki; }

    void reset();

    // --- Debug / Monitoring ---
    float lastError() const { return _lastE; }
    float lastP()     const { return _lastP; }
    float lastI()     const { return _lastI; }

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

    // Debug
    float _lastE;
    float _lastP;
    float _lastI;
};