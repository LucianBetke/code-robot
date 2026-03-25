// ============================================================
// File: VehicleController.cpp
// ============================================================

#include "VehicleController.h"
#include "ControlParams.h"
#include <math.h>

VehicleController::VehicleController(Motor& motorRe, Motor& motorLi)
    : _motorRe(motorRe), _motorLi(motorLi) {
}

void VehicleController::selectSide(Side s) {
    _side = s;
}

void VehicleController::setPWM(int16_t pwm) {
    _pwm = pwm;
}
void VehicleController::setCmd(float vx, float vy, float omega) {
    _cmd.vx = vx;
    _cmd.vy = vy;
    _cmd.omega = omega;
    mix2W();
}

void VehicleController::mix2W() {
    // 2-Rad-Hinterachse:
    // vR = vx + (W/2)*omega
    // vL = vx - (W/2)*omega
    //
    // Hinweis: vy ist in 2-Rad-Konfiguration nicht darstellbar und wird ignoriert.

    const float halfW = 0.5f * ACHS_ABSTAND_M;   // W = Spurweite (m)
    float vR = _cmd.vx + halfW * _cmd.omega;
    float vL = _cmd.vx - halfW * _cmd.omega;

    // Gemeinsame Sättigung: |vR| und |vL| dürfen nicht > V_SOLL_GERADE werden.
    const float aR = fabsf(vR);
    const float aL = fabsf(vL);
    const float aMax = (aR > aL) ? aR : aL;

    if (aMax > V_SOLL_GERADE) {
        const float k = V_SOLL_GERADE / aMax;
        vR *= k;
        vL *= k;
    }

    _vSollRe = vR;
    _vSollLi = vL;
}

float VehicleController::getWheelSollMps(Side s) const {
    if (s == Side::RE) return _vSollRe;
    if (s == Side::LI) return _vSollLi;
    return 0.0f;
}

void VehicleController::setCmdNorm(float ux, float uy, float uOmega) {
    // 1) (ux,uy) Joystick-Normierung: diagonal nicht schneller als max
    float uxN = ux;
    float uyN = uy;
    const float r2 = uxN * uxN + uyN * uyN;
    if (r2 > 1.0f) {
        const float invr = 1.0f / sqrtf(r2);
        uxN *= invr;
        uyN *= invr;
    }

    // 2) Normiert -> physikalisch
    //    vx/vy in m/s, omega in rad/s
    const float vx = uxN * V_SOLL_GERADE;
    const float vy = uyN * V_SOLL_GERADE;
    const float omega = uOmega * OMEGA_REF;

    // 3) Weiterverarbeitung über den bestehenden Pfad
    setCmd(vx, vy, omega);
}

void VehicleController::update() {
    // signed PWM auswerten
    const int16_t pwm = _pwm;
    const uint8_t apwm = (pwm >= 0) ? (uint8_t)pwm : (uint8_t)(-pwm);

    // 0 => bremsen
    if (apwm == 0) {
        _motorRe.bremse(HIGH);
        _motorLi.bremse(HIGH);
        return;
    }

    // Helper-Lambda: wendet Richtung auf einen Motor an
    auto drive = [&](Motor& m) {
        if (pwm > 0) m.vor(apwm);
        else         m.rueck(apwm);
        };

    if (_side == Side::RE) {
        drive(_motorRe);
        _motorLi.bremse(HIGH);
    }
    else if (_side == Side::LI) {
        drive(_motorLi);
        _motorRe.bremse(HIGH);
    }
    else if (_side == Side::BOTH) {
        drive(_motorRe);
        drive(_motorLi);
    }
}
