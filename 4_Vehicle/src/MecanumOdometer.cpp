// ============================================================
// File: MecanumOdometer.cpp
// ============================================================

#include "MecanumOdometer.h"

#include <math.h>

MecanumOdometer::MecanumOdometer()
    : _primed(false),
    _lastCounts{ 0, 0, 0, 0 },
    _x_mm(0.0f),
    _y_mm(0.0f),
    _abs_mm(0.0f),
    _phi_rad(0.0f)
{
}

void MecanumOdometer::reset(const WheelCounts& counts)
{
    _primed = true;

    for (uint8_t i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        _lastCounts[i] = (int32_t)counts.v[i];
    }

    _x_mm = 0.0f;
    _y_mm = 0.0f;
    _abs_mm = 0.0f;
    _phi_rad = 0.0f;
}

bool MecanumOdometer::update(const WheelCounts& counts)
{
    if (!_primed)
    {
        reset(counts);
        return false;
    }

    int32_t dCounts[WHEEL_VEHICLE_COUNT];

    for (uint8_t i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        dCounts[i] = (int32_t)counts.v[i] - _lastCounts[i];
        _lastCounts[i] = (int32_t)counts.v[i];
    }

    const float dVoRe = countsToMm(dCounts[VoRe]);
    const float dVoLi = countsToMm(dCounts[VoLi]);
    const float dHiLi = countsToMm(dCounts[HiLi]);
    const float dHiRe = countsToMm(dCounts[HiRe]);

    // Vorwaertskinematik passend zu VehicleController::applyMixer():
    //
    // vVoRe = vx + vy + k*wz
    // vVoLi = vx - vy - k*wz
    // vHiLi = vx + vy - k*wz
    // vHiRe = vx - vy + k*wz
    //
    // Daraus:
    // dx   = (VoRe + VoLi + HiLi + HiRe) / 4
    // dy   = (VoRe - VoLi + HiLi - HiRe) / 4
    // dphi = (VoRe - VoLi - HiLi + HiRe) / (4*k)

    const float dx_body_mm =
        (dVoRe + dVoLi + dHiLi + dHiRe) * 0.25f;

    const float dy_body_mm =
        (dVoRe - dVoLi + dHiLi - dHiRe) * 0.25f;

    const float dphi_rad =
        (dVoRe - dVoLi - dHiLi + dHiRe) / (4.0f * MECANUM_K_MM);

    // Speicheroptimierung:
    // Keine Weltkoordinaten-Transformation mehr auf dem Arduino.
    //
    // Der Arduino summiert nur die lokale Roboterbewegung.
    // Die Umrechnung in Weltkoordinaten uebernimmt Python.
    //
    // Wichtig:
    //   _phi_rad bleibt erhalten, damit Python oder eine spaetere Regelung
    //   die Roboterverdrehung weiterhin kennt.

    _x_mm += dx_body_mm;
    _y_mm += dy_body_mm;
    _phi_rad += dphi_rad;

    // sqrtf bleibt vorerst erhalten.
    // absMm() / absCm() werden weiterhin fuer CMDP / ODOM2 gebraucht.
    _abs_mm = sqrtf(_x_mm * _x_mm + _y_mm * _y_mm);

    return true;
}

float MecanumOdometer::countsToMm(int32_t dCounts) const
{
    return ((float)dCounts / (float)COUNTS_PER_REV) * RAD_UMFANG_MM;
}

float MecanumOdometer::phiDeg() const
{
    return _phi_rad * 57.2957795f;
}