// Rad.cpp
#include "Rad.h"
#include <math.h>            // fabsf

namespace
{
    const float EPS = 1e-6f;

    int16_t roundPwm(float pwm)
    {
        if (pwm > (float)MAX_PWM)  pwm = (float)MAX_PWM;
        if (pwm < (float)-MAX_PWM) pwm = (float)-MAX_PWM;

        if (pwm >= 0.0f) pwm += 0.5f;
        else             pwm -= 0.5f;

        return (int16_t)pwm;
    }

    float scaledWorkpointWithDeadPwm(
        float v_alt,
        float v_neu,
        int16_t oldPwm,
        int16_t deadPwm)
    {
        const float sign = (v_neu >= 0.0f) ? 1.0f : -1.0f;

        int16_t deadAbs = (deadPwm >= 0) ? deadPwm : (int16_t)-deadPwm;
        if (deadAbs > MAX_PWM) deadAbs = MAX_PWM;

        int16_t oldAbs = (oldPwm >= 0) ? oldPwm : (int16_t)-oldPwm;
        if (oldAbs > MAX_PWM) oldAbs = MAX_PWM;

        const bool oldPwmInNewDirection =
            ((v_neu > 0.0f) && (oldPwm > 0)) ||
            ((v_neu < 0.0f) && (oldPwm < 0));

        if (!oldPwmInNewDirection || oldAbs < deadAbs)
        {
            oldAbs = deadAbs;
        }

        const float ratio = fabsf(v_neu) / fabsf(v_alt);

        float dynamicOld = (float)oldAbs - (float)deadAbs;
        if (dynamicOld < 0.0f) dynamicOld = 0.0f;

        float newAbs = (float)deadAbs + dynamicOld * ratio;

        if (newAbs > (float)MAX_PWM) newAbs = (float)MAX_PWM;
        if (newAbs < (float)deadAbs) newAbs = (float)deadAbs;

        return sign * newAbs;
    }
}

Rad::Rad(Motor& motor, SpeedWeg& speed, PIRegler& regler,
    uint16_t dtMs, int16_t deadPwm)
    : _motor(motor), _speed(speed), _regler(regler),
    _dtMs(dtMs), _deadPwm(deadPwm),
    _lastUpdateMs(0),
    _lastPwm(0)
{
}

void Rad::setSoll(float v_soll)
{
    const float v_alt = _regler.soll();

    const bool wasZero = (fabsf(v_alt) < EPS);
    const bool nowZero = (fabsf(v_soll) < EPS);
    const bool changed = (fabsf(v_soll - v_alt) >= EPS);

    // Harter Stop sofort beim Setzen des Sollwerts 0
    if (nowZero) {
        _regler.reset();
        _regler.setSoll(0.0f);
        _motor.bremse(HIGH);
        _lastPwm = 0;
        _lastUpdateMs = 0;
        return;
    }

    // Wenn der Sollwert unverändert ist:
    // nichts am Integrator ändern.
    // Wichtig, weil setSoll() im Hauptprogramm sehr häufig aufgerufen wird.
    if (!changed) {
        _regler.setSoll(v_soll);
        return;
    }

    const bool dirFlip =
        (!wasZero) &&
        ((v_alt > 0.0f && v_soll < 0.0f) ||
            (v_alt < 0.0f && v_soll > 0.0f));

    // Neuer Sollwert zuerst setzen.
    _regler.setSoll(v_soll);

    // Fall 1:
    // Start aus Stillstand oder Richtungswechsel.
    // Hier wird nicht skaliert, sondern ein sauberer Startwert in neuer Richtung gesetzt.
    if (wasZero || dirFlip) {
        const float sign = (v_soll >= 0.0f) ? 1.0f : -1.0f;
        _regler.presetOutput(sign * (float)_deadPwm);

        _lastUpdateMs = 0;
        _lastPwm = 0;
        return;
    }

    // Fall 2:
    // Gleiche Richtung, aber neue Geschwindigkeit.
    //
    // Beispiel:
    // 30 -> 20 oder -30 -> -20
    //
    // Der alte Arbeitspunkt wird nicht komplett behalten.
    // Er wird deadPWM-korrigiert auf die neue Geschwindigkeit skaliert:
    //
    // pwm_neu = deadPwm + (pwm_alt - deadPwm) * |v_neu| / |v_alt|
    //
    // Dadurch bleibt der Reibungs-/Anfahranteil erhalten,
    // aber der dynamische Anteil wird an die neue Geschwindigkeit angepasst.
    {
        const float u0_pwm =
            scaledWorkpointWithDeadPwm(v_alt, v_soll, _lastPwm, _deadPwm);

        _regler.presetOutput(u0_pwm);

        // _lastUpdateMs absichtlich NICHT zurücksetzen.
        // Die Regelung soll ohne künstliche Pause weiterlaufen.
        _lastPwm = roundPwm(u0_pwm);
    }
}

float Rad::soll() const
{
    return _regler.soll();
}

float Rad::vIst() const
{
    return _speed.mps();
}

void Rad::stop()
{
    _regler.reset();
    _regler.setSoll(0.0f);
    _motor.bremse(HIGH);
    _lastPwm = 0;
    _lastUpdateMs = 0;
}

void Rad::update(uint32_t nowMs)
{
    _speed.update(nowMs);

    const float v_soll = _regler.soll();

    // STOP muss VOR der Zeitabfrage kommen.
    // Sonst kann ein frisch gesetzter Stop-Befehl bis zum nächsten Regeltakt verzögert werden.
    if (fabsf(v_soll) < EPS) {
        _regler.reset();
        _motor.bremse(HIGH);
        _lastPwm = 0;
        _lastUpdateMs = 0;
        return;
    }

    if (_lastUpdateMs == 0) {
        _lastUpdateMs = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - _lastUpdateMs) < _dtMs) {
        return;
    }

    const uint16_t dt_ms = (uint16_t)(nowMs - _lastUpdateMs);
    _lastUpdateMs = nowMs;

    const float v_ist = _speed.mps();

    int16_t pwm = _regler.update(v_ist, dt_ms);

    // Limit
    if (pwm > MAX_PWM)  pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    // Deadband
    int16_t apwm = (pwm >= 0) ? pwm : -pwm;

    if (apwm > 0 && apwm < _deadPwm) {
        const bool sollVor = (v_soll > EPS);
        const bool sollRueck = (v_soll < -EPS);
        const bool pwmVor = (pwm > 0);
        const bool pwmRueck = (pwm < 0);

        const bool pwmInSollrichtung =
            (sollVor && pwmVor) ||
            (sollRueck && pwmRueck);

        if (pwmInSollrichtung) {
            pwm = pwmVor ? _deadPwm : -_deadPwm;
            apwm = _deadPwm;
        }
        else {
            pwm = 0;
            apwm = 0;
        }
    }

    // Sicherheit: falls _deadPwm versehentlich groesser als MAX_PWM gesetzt wurde
    if (pwm > MAX_PWM)  pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    apwm = (pwm >= 0) ? pwm : -pwm;

    // Motor
    if (pwm > 0) {
        _motor.vor((uint8_t)apwm);
    }
    else if (pwm < 0) {
        _motor.rueck((uint8_t)apwm);
    }
    else {
        _motor.bremse(HIGH);
    }

    _lastPwm = pwm;
}

void Rad::reset()
{
    _regler.reset();
    _regler.setSoll(0.0f);
    _lastUpdateMs = 0;
    _lastPwm = 0;
}

// DeadPWM zur Laufzeit neu setzen
void Rad::setDeadPwm(int16_t deadPwm)
{
    _deadPwm = deadPwm;
}