// ============================================================
// VehicleController.cpp
// Koordinatenkonvention:
// +x  = vorwärts
// +y  = links
// +z  = oben
// +wz = Drehung gegen den Uhrzeigersinn, von oben betrachtet
// ============================================================

#include "VehicleController.h"
#include <math.h>

static const float VEHICLE_WZ_EPS = 0.0001f;

void VehicleController::begin(float Kp_vx, float Ki_vx,
    float Kp_vy, float Ki_vy,
    float Kp_wz, float Ki_wz)
{
    _regler.setParams(Kp_vx, Ki_vx, Kp_vy, Ki_vy, Kp_wz, Ki_wz);
    _regler.reset();
    _lastUpdateMs = 0;
    _turnOnly = false;
}

void VehicleController::cmd(float vx, float vy, float wz)
{
    applyDriveMode(vx, vy, wz);

    if (!_turnOnly)
    {
        limitTranslation(vx, vy);
    }

    _vx = vx;
    _vy = vy;
    _wz = wz;

    applyMixer(_vx, _vy, _wz);
}

void VehicleController::applyDriveMode(float& vx, float& vy, float wz)
{
    _turnOnly = (fabsf(wz) > VEHICLE_WZ_EPS);

    if (_turnOnly)
    {
        vx = 0.0f;
        vy = 0.0f;
    }
}

void VehicleController::limitTranslation(float& vx, float& vy)
{
    const float sum = fabsf(vx) + fabsf(vy);

    if (sum <= V_WHEEL_MAX)
    {
        return;
    }

    if (sum <= 0.0001f)
    {
        vx = 0.0f;
        vy = 0.0f;
        return;
    }

    const float scale = V_WHEEL_MAX / sum;

    vx *= scale;
    vy *= scale;
}

void VehicleController::updateIst(float v0, float v1, float v2, float v3)
{
    _vx_ist = (v0 + v1 + v2 + v3) / 4.0f;

    // +y = links
    _vy_ist = (v0 - v1 + v2 - v3) / 4.0f;

    // +wz = gegen Uhrzeigersinn, +z nach oben
    _wz_ist = (v0 - v1 - v2 + v3) / (4.0f * MECANUM_K);
}

void VehicleController::update(uint32_t now)
{
    if (_lastUpdateMs == 0)
    {
        _lastUpdateMs = now;
        applyMixer(_vx, _vy, _wz);
        return;
    }

    if ((uint32_t)(now - _lastUpdateMs) < VEHICLE_DT_MS) return;

    uint16_t dt_ms = (uint16_t)(now - _lastUpdateMs);
    _lastUpdateMs = now;

    float vx_korr = 0.0f;
    float vy_korr = 0.0f;
    float wz_korr = 0.0f;

    if (_turnOnly)
    {
        vx_korr = 0.0f;
        vy_korr = 0.0f;
        wz_korr = _regler.updateWz(_wz, _wz_ist, dt_ms);
    }
    else
    {
        vx_korr = _regler.updateVx(_vx, _vx_ist, dt_ms);
        vy_korr = _regler.updateVy(_vy, _vy_ist, dt_ms);
        wz_korr = _regler.updateWz(_wz, _wz_ist, dt_ms);

        limitTranslation(vx_korr, vy_korr);
    }

    applyMixer(vx_korr, vy_korr, wz_korr);
}

void VehicleController::applyMixer(float vx, float vy, float wz)
{
    float v[WHEEL_VEHICLE_COUNT];

    // Inverse Kinematik:
    // +x  = vorwärts
    // +y  = links
    // +wz = gegen Uhrzeigersinn
    v[VoRe] = vx + vy + MECANUM_K * wz;
    v[VoLi] = vx - vy - MECANUM_K * wz;
    v[HiLi] = vx + vy - MECANUM_K * wz;
    v[HiRe] = vx - vy + MECANUM_K * wz;

    float maxVal = 0.0f;

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        float a = fabsf(v[i]);
        if (a > maxVal) maxVal = a;
    }

    if (maxVal > V_WHEEL_MAX && maxVal > 0.0001f)
    {
        float scale = V_WHEEL_MAX / maxVal;

        for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
        {
            v[i] *= scale;
        }
    }

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        _wheelSoll[i] = v[i];
    }
}

void VehicleController::stop()
{
    _vx = 0.0f;
    _vy = 0.0f;
    _wz = 0.0f;

    _turnOnly = false;

    _regler.reset();

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        _wheelSoll[i] = 0.0f;
    }
}

float VehicleController::getWheelSoll(WheelVehicle w) const
{
    return _wheelSoll[w];
}