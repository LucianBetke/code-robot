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
    _phi_rad(0.0f),
    _last_dx_body_mm(0.0f),
    _last_dy_body_mm(0.0f),
    _last_dphi_rad(0.0f)
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

    _last_dx_body_mm = 0.0f;
    _last_dy_body_mm = 0.0f;
    _last_dphi_rad = 0.0f;
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

    // Vorwaertskinematik fuer Mecanum:
    // +x  = vorwaerts
    // +y  = links
    // +phi = Drehung gegen Uhrzeigersinn
    const float dx_body_mm =
        (dVoRe + dVoLi + dHiLi + dHiRe) * 0.25f;

    const float dy_body_mm =
        (dVoRe - dVoLi + dHiLi - dHiRe) * 0.25f;

    const float dphi_rad =
        (dVoRe - dVoLi - dHiLi + dHiRe) / (4.0f * MECANUM_K_MM);

    _last_dx_body_mm = dx_body_mm;
    _last_dy_body_mm = dy_body_mm;
    _last_dphi_rad = dphi_rad;

    // Koerperkoordinaten in Weltkoordinaten drehen.
    // Bei CMDP Version 1 ist wz = 0, trotzdem ist das hier schon korrekt
    // fuer spaetere Drehbewegungen vorbereitet.
    const float phi_mid = _phi_rad + 0.5f * dphi_rad;
    const float c = cosf(phi_mid);
    const float s = sinf(phi_mid);

    const float dx_world_mm = dx_body_mm * c - dy_body_mm * s;
    const float dy_world_mm = dx_body_mm * s + dy_body_mm * c;

    _x_mm += dx_world_mm;
    _y_mm += dy_world_mm;
    _phi_rad += dphi_rad;

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