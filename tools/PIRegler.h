// PIRegler.h
#pragma once
#include <Arduino.h>   // int16_t, uint16_t, constrain

class PIRegler {
public:
    PIRegler(float Kp, float Ki, int16_t uMin, int16_t uMax);

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
    float lastP()     const { return _lastP; }   // Kp * e (in u_norm)
    float lastI()     const { return _lastI; }   // Integralanteil (in u_norm)

private:
    float   _Kp;
    float   _Ki;
    float   _v_soll;
    float   _integral;    // in u_norm
    int16_t _uMin;        // PWM-Limits
    int16_t _uMax;

    int16_t _uPrev;       // letzte ausgegebene PWM (für Slew-Rate)

    int16_t applySlew(int16_t pwm);

    // Debug
    float _lastE;
    float _lastP;
    float _lastI;
};
