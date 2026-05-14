// Rad.cpp
#include "Rad.h"
#include <math.h>            // fabsf

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
    _regler.setSoll(v_soll);

    const float EPS = 1e-6f;
    const bool wasZero = (fabsf(v_alt) < EPS);
    const bool nowZero = (fabsf(v_soll) < EPS);

    if (!nowZero) {
        const bool dirFlip =
            (!wasZero) &&
            ((v_alt > 0.0f && v_soll < 0.0f) ||
                (v_alt < 0.0f && v_soll > 0.0f));

        if (wasZero || dirFlip) {
            const float sign = (v_soll >= 0.0f) ? 1.0f : -1.0f;
            _regler.presetOutput(sign * (float)_deadPwm);

            _lastUpdateMs = 0;
            _lastPwm = 0;
        }
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

    if (_lastUpdateMs == 0) {
        _lastUpdateMs = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - _lastUpdateMs) < _dtMs) {
        return;
    }

    const uint16_t dt_ms = (uint16_t)(nowMs - _lastUpdateMs);
    _lastUpdateMs = nowMs;

    const float EPS = 1e-6f;
    const float v_soll = _regler.soll();

    // STOP
    if (fabsf(v_soll) < EPS) {
        _regler.reset();
        _motor.bremse(HIGH);
        _lastPwm = 0;
        return;
    }

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

PIParam Rad::getPI() const
{
    return { _regler.Kp(), _regler.Ki() };
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