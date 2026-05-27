// ============================================================
// File: MecanumOdometer.cpp
// ============================================================

#include "MecanumOdometer.h"

#include <math.h>
#include "src/globals.h"

MecanumOdometer::MecanumOdometer()
    : _primed(false),
    _lastVoReCnt(0),
    _lastVoLiCnt(0),
    _lastHiLiCnt(0),
    _lastHiReCnt(0),
    _x_mm(0.0f),
    _y_mm(0.0f),
    _abs_mm(0.0f),
    _phi_rad(0.0f)
{
}

void MecanumOdometer::reset(
    int32_t voReCnt,
    int32_t voLiCnt,
    int32_t hiLiCnt,
    int32_t hiReCnt)
{
    _primed = true;

    _lastVoReCnt = voReCnt;
    _lastVoLiCnt = voLiCnt;
    _lastHiLiCnt = hiLiCnt;
    _lastHiReCnt = hiReCnt;

    _x_mm = 0.0f;
    _y_mm = 0.0f;
    _abs_mm = 0.0f;
    _phi_rad = 0.0f;
}

bool MecanumOdometer::update(
    int32_t voReCnt,
    int32_t voLiCnt,
    int32_t hiLiCnt,
    int32_t hiReCnt)
{
    if (!_primed)
    {
        reset(voReCnt, voLiCnt, hiLiCnt, hiReCnt);
        return false;
    }

    const int32_t dVoReCnt = voReCnt - _lastVoReCnt;
    const int32_t dVoLiCnt = voLiCnt - _lastVoLiCnt;
    const int32_t dHiLiCnt = hiLiCnt - _lastHiLiCnt;
    const int32_t dHiReCnt = hiReCnt - _lastHiReCnt;

    _lastVoReCnt = voReCnt;
    _lastVoLiCnt = voLiCnt;
    _lastHiLiCnt = hiLiCnt;
    _lastHiReCnt = hiReCnt;

    const float dVoRe = countsToMm(dVoReCnt);
    const float dVoLi = countsToMm(dVoLiCnt);
    const float dHiLi = countsToMm(dHiLiCnt);
    const float dHiRe = countsToMm(dHiReCnt);

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