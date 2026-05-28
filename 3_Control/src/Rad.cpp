// Rad.cpp
#include "Rad.h"
#include <math.h>

namespace
{
    const float EPS = 0.001f;

    int16_t roundPwm(float pwm)
    {
        if (pwm > (float)MAX_PWM)  pwm = (float)MAX_PWM;
        if (pwm < (float)-MAX_PWM) pwm = (float)-MAX_PWM;

        if (pwm >= 0.0f) pwm += 0.5f;
        else             pwm -= 0.5f;

        return (int16_t)pwm;
    }

    float scaledWorkpointWithDeadPwm(
        float v_alt_cms,
        float v_neu_cms,
        int16_t oldPwm,
        int16_t deadPwm)
    {
        const float sign = (v_neu_cms >= 0.0f) ? 1.0f : -1.0f;

        int16_t deadAbs = (deadPwm >= 0) ? deadPwm : (int16_t)-deadPwm;
        if (deadAbs > MAX_PWM) deadAbs = MAX_PWM;

        int16_t oldAbs = (oldPwm >= 0) ? oldPwm : (int16_t)-oldPwm;
        if (oldAbs > MAX_PWM) oldAbs = MAX_PWM;

        const bool oldPwmInNewDirection =
            ((v_neu_cms > 0.0f) && (oldPwm > 0)) ||
            ((v_neu_cms < 0.0f) && (oldPwm < 0));

        if (!oldPwmInNewDirection || oldAbs < deadAbs)
        {
            oldAbs = deadAbs;
        }

        const float ratio = fabsf(v_neu_cms) / fabsf(v_alt_cms);

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

void Rad::setSoll(float v_soll_cms)
{
    const float v_alt_cms = _regler.soll();

    const bool wasZero = (fabsf(v_alt_cms) < EPS);
    const bool nowZero = (fabsf(v_soll_cms) < EPS);
    const bool changed = (fabsf(v_soll_cms - v_alt_cms) >= EPS);

    if (nowZero)
    {
        _regler.reset();
        _regler.setSoll(0.0f);
        _motor.bremse(HIGH);
        _lastPwm = 0;
        _lastUpdateMs = 0;
        return;
    }

    if (!changed)
    {
        _regler.setSoll(v_soll_cms);
        return;
    }

    const bool dirFlip =
        (!wasZero) &&
        ((v_alt_cms > 0.0f && v_soll_cms < 0.0f) ||
            (v_alt_cms < 0.0f && v_soll_cms > 0.0f));

    _regler.setSoll(v_soll_cms);

    if (wasZero || dirFlip)
    {
        const float sign = (v_soll_cms >= 0.0f) ? 1.0f : -1.0f;
        _regler.presetOutput(sign * (float)_deadPwm);

        _lastUpdateMs = 0;
        _lastPwm = 0;
        return;
    }

    {
        const float u0_pwm =
            scaledWorkpointWithDeadPwm(v_alt_cms, v_soll_cms, _lastPwm, _deadPwm);

        _regler.presetOutput(u0_pwm);

        _lastPwm = roundPwm(u0_pwm);
    }
}

float Rad::soll() const
{
    return _regler.soll();
}

float Rad::vIst() const
{
    return _speed.cms();
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

    const float v_soll_cms = _regler.soll();

    if (fabsf(v_soll_cms) < EPS)
    {
        _regler.reset();
        _motor.bremse(HIGH);
        _lastPwm = 0;
        _lastUpdateMs = 0;
        return;
    }

    if (_lastUpdateMs == 0)
    {
        _lastUpdateMs = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - _lastUpdateMs) < _dtMs)
    {
        return;
    }

    const uint16_t dt_ms = (uint16_t)(nowMs - _lastUpdateMs);
    _lastUpdateMs = nowMs;

    const float v_ist_cms = _speed.cms();

    int16_t pwm = _regler.update(v_ist_cms, dt_ms);

    if (pwm > MAX_PWM)  pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    int16_t apwm = (pwm >= 0) ? pwm : -pwm;

    if (apwm > 0 && apwm < _deadPwm)
    {
        const bool sollVor = (v_soll_cms > EPS);
        const bool sollRueck = (v_soll_cms < -EPS);
        const bool pwmVor = (pwm > 0);
        const bool pwmRueck = (pwm < 0);

        const bool pwmInSollrichtung =
            (sollVor && pwmVor) ||
            (sollRueck && pwmRueck);

        if (pwmInSollrichtung)
        {
            pwm = pwmVor ? _deadPwm : -_deadPwm;
            apwm = _deadPwm;
        }
        else
        {
            pwm = 0;
            apwm = 0;
        }
    }

    if (pwm > MAX_PWM)  pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    apwm = (pwm >= 0) ? pwm : -pwm;

    if (pwm > 0)
    {
        _motor.vor((uint8_t)apwm);
    }
    else if (pwm < 0)
    {
        _motor.rueck((uint8_t)apwm);
    }
    else
    {
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

void Rad::setDeadPwm(int16_t deadPwm)
{
    _deadPwm = deadPwm;
}