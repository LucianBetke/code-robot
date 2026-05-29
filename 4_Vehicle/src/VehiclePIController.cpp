// ============================================================
// VehiclePIController.cpp
// ============================================================
#include "VehiclePIController.h"

VehicleRegler::VehicleRegler()
    : _Kp_vx(0.0f), _Ki_vx(0.0f)
    , _Kp_vy(0.0f), _Ki_vy(0.0f)
    , _Kp_wz(0.0f), _Ki_wz(0.0f)
    , _integral_vx(0.0f)
    , _integral_vy(0.0f)
    , _integral_wz(0.0f)
{
}

void VehicleRegler::setParams(float Kp_vx, float Ki_vx,
    float Kp_vy, float Ki_vy,
    float Kp_wz, float Ki_wz)
{
    _Kp_vx = Kp_vx; _Ki_vx = Ki_vx;
    _Kp_vy = Kp_vy; _Ki_vy = Ki_vy;
    _Kp_wz = Kp_wz; _Ki_wz = Ki_wz;
}

void VehicleRegler::reset()
{
    _integral_vx = 0.0f;
    _integral_vy = 0.0f;
    _integral_wz = 0.0f;
}

float VehicleRegler::updateVx(float soll, float ist, uint16_t dt_ms)
{
    if (_Kp_vx == 0.0f && _Ki_vx == 0.0f) return soll;
    if (dt_ms == 0) return soll;
    const float dt_s = dt_ms * 0.001f;
    const float e = soll - ist;
    _integral_vx += _Ki_vx * e * dt_s;
    return _Kp_vx * e + _integral_vx;
}

float VehicleRegler::updateVy(float soll, float ist, uint16_t dt_ms)
{
    if (_Kp_vy == 0.0f && _Ki_vy == 0.0f) return soll;
    if (dt_ms == 0) return soll;
    const float dt_s = dt_ms * 0.001f;
    const float e = soll - ist;
    _integral_vy += _Ki_vy * e * dt_s;
    return _Kp_vy * e + _integral_vy;
}

float VehicleRegler::updateWz(float soll, float ist, uint16_t dt_ms)
{
    if (_Kp_wz == 0.0f && _Ki_wz == 0.0f) return soll;
    if (dt_ms == 0) return soll;
    const float dt_s = dt_ms * 0.001f;
    const float e = soll - ist;
    _integral_wz += _Ki_wz * e * dt_s;
    return _Kp_wz * e + _integral_wz;
}