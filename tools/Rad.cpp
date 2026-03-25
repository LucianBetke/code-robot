// Rad.cpp
#include "Rad.h"
#include "ControlParams.h" // DEAD_PWM, MAX_PWM
#include <math.h>          // fabsf

Rad::Rad(Motor& motor, SpeedWeg& speed, PIRegler& regler, uint16_t dtMs, Wheel radIndex)
    : _motor(motor), _speed(speed), _regler(regler), _dtMs(dtMs), _lastUpdateMs(0),
    _lastPwm(0), _trim(0), _index(radIndex)
{}

void Rad::setTrim(int16_t t) {_trim = t;}

void Rad::setSoll(float v_soll) {
    const float v_alt = _regler.soll();
    _regler.setSoll(v_soll);

    const float EPS = 1e-6f;

    const bool wasZero = (fabsf(v_alt) < EPS);
    const bool nowZero = (fabsf(v_soll) < EPS);

    // Startimpuls:
    //  - von 0 -> !=0 : Kick in die richtige Richtung
    //  - Richtungswechsel: ebenfalls neu starten (sauberer Übergang)
    if (!nowZero) {
        const bool dirFlip = (!wasZero) && ((v_alt > 0.0f && v_soll < 0.0f) || (v_alt < 0.0f && v_soll > 0.0f));
        if (wasZero || dirFlip) {
            const float sign = (v_soll >= 0.0f) ? 1.0f : -1.0f;
            _regler.presetOutput(sign * (float)DEAD_PWM[_index]);

            // Zeitbasis neu setzen, damit der Regler sauber startet
            _lastUpdateMs = 0;
            _lastPwm = 0;
        }
    }
    else {
        // Soll = 0: beim nächsten update() wird gebremst (siehe unten)
        // (keine harte Regler-Manipulation hier nötig)
    }
}

float Rad::soll() const {
    return _regler.soll();
}

void Rad::update(uint32_t nowMs) {
    // erste Initialisierung: Zeitbasis setzen und noch NICHT regeln
    if (_lastUpdateMs == 0) {
        _lastUpdateMs = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - _lastUpdateMs) < (uint32_t)_dtMs) {
        return;
    }

    const uint16_t dt_ms = (uint16_t)(nowMs - _lastUpdateMs);
    _lastUpdateMs = nowMs;

    const float EPS = 1e-6f;
    const float v_soll = _regler.soll();

    // Soll ~ 0: Motor hart bremsen und sauber 0 ausgeben
    if (fabsf(v_soll) < EPS) {
        _motor.bremse(HIGH);
        _lastPwm = 0;
        return;
    }

    // Ist-Geschwindigkeit (m/s)
    const float v_ist = _speed.mps();

    // PI-Regler (muss signed liefern dürfen)
    int16_t pwm = _regler.update(v_ist, dt_ms);

    // Fahrtrichtung aus Soll (nicht aus pwm!): vorwärts +1, rückwärts -1
    const int16_t dir = (v_soll >= 0.0f) ? (int16_t)+1 : (int16_t)-1;

    // Trim überlagern, aber richtungsrichtig
    pwm = (int16_t)(pwm + (int16_t)(dir * _trim));


    // Sättigung symmetrisch: -MAX_PWM..+MAX_PWM
    if (pwm > (int16_t)MAX_PWM) pwm = (int16_t)MAX_PWM;
    if (pwm < (int16_t)-MAX_PWM) pwm = (int16_t)-MAX_PWM;

    // Deadband symmetrisch (nur wenn Soll != 0):
    // Betrag in (0..DEAD_PWM) hochsetzen, Vorzeichen beibehalten.

    int16_t apwm = (pwm >= 0) ? pwm : (int16_t)(-pwm);
    const int16_t dead = DEAD_PWM[_index];

    if (apwm > 0 && apwm < dead) {
        pwm = (pwm >= 0) ? dead : (int16_t)(-dead);
        apwm = dead;
    }

    // Stellgröße an Motor: Richtung aus Vorzeichen
    if (pwm > 0) {
        _motor.vor((uint8_t)apwm);
    }
    else if (pwm < 0) {
        _motor.rueck((uint8_t)apwm);
    }
    else {
        _motor.bremse(HIGH);
    }

    _lastPwm = pwm; // signed speichern (Debug/Logging)
}

PIParam Rad::getPI() const
{
    return { _regler.Kp(), _regler.Ki() };
}

void Rad::reset() {
    _regler.reset();
    _regler.setSoll(0.0f);   // <<< ENTSCHEIDEND
    _lastUpdateMs = 0;
    _lastPwm = 0;
    _trim = 0;               // optional, aber sinnvoll
}

