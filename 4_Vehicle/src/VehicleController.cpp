// ============================================================
// VehicleController.cpp
// Koordinatenkonvention:
// +x  = vorwaerts
// +y  = links
// +z  = oben
// +wz = Drehung gegen den Uhrzeigersinn, von oben betrachtet
//
// Einheiten:
// vx, vy, Radgeschwindigkeiten: cm/s
// wz: rad/s
// ============================================================

#include "VehicleController.h"
#include "MecanumKinematics.h"

#include <math.h>

static const float VEHICLE_WZ_EPS = 0.0001f;

void VehicleController::begin(float Kp_vx, float Ki_vx,
    float Kp_vy, float Ki_vy,
    float Kp_wz, float Ki_wz)
{
    _regler.setParams(Kp_vx, Ki_vx, Kp_vy, Ki_vy, Kp_wz, Ki_wz);
    _regler.reset();

    _chassis.reset();

    _lastUpdateMs = 0;
    _turnOnly = false;
}

void VehicleController::begin(const PIParam& vx, const PIParam& vy, const PIParam& wz)
{
    begin(
        vx.Kp, vx.Ki,
        vy.Kp, vy.Ki,
        wz.Kp, wz.Ki
    );
}

void VehicleController::begin(const VehicleControlConfig& cfg)
{
    begin(cfg.vx, cfg.vy, cfg.wz);
}

void VehicleController::cmd(float vx_cms, float vy_cms, float wz_rad_s)
{
    applyDriveMode(vx_cms, vy_cms, wz_rad_s);

    if (!_turnOnly)
    {
        MecanumKinematics::limitTranslation(vx_cms, vy_cms);
    }

    _vx = vx_cms;
    _vy = vy_cms;
    _wz = wz_rad_s;

    float vx_chassis = 0.0f;
    float vy_chassis = 0.0f;
    float wz_chassis = 0.0f;

    _chassis.update(
        _vx,
        _vy,
        _wz,
        vx_chassis,
        vy_chassis,
        wz_chassis
    );

    if (!_turnOnly)
    {
        MecanumKinematics::limitTranslation(vx_chassis, vy_chassis);
    }

    applyMixer(vx_chassis, vy_chassis, wz_chassis);
}

void VehicleController::applyDriveMode(float& vx_cms, float& vy_cms, float wz_rad_s)
{
    _turnOnly = (fabsf(wz_rad_s) > VEHICLE_WZ_EPS);

    if (_turnOnly)
    {
        vx_cms = 0.0f;
        vy_cms = 0.0f;
    }
}

void VehicleController::updateIst(const WheelSpeedCms& wheelIst)
{
    MecanumKinematics::forward(
        wheelIst,
        _vx_ist,
        _vy_ist,
        _wz_ist
    );
}

void VehicleController::updateChassisState(const ChassisState& state)
{
    _chassis.updateState(state);
}

void VehicleController::update(uint32_t now)
{
    if (_lastUpdateMs == 0)
    {
        _lastUpdateMs = now;

        float vx_chassis = 0.0f;
        float vy_chassis = 0.0f;
        float wz_chassis = 0.0f;

        _chassis.update(
            _vx,
            _vy,
            _wz,
            vx_chassis,
            vy_chassis,
            wz_chassis
        );

        if (!_turnOnly)
        {
            MecanumKinematics::limitTranslation(vx_chassis, vy_chassis);
        }

        applyMixer(vx_chassis, vy_chassis, wz_chassis);
        return;
    }

    if ((uint32_t)(now - _lastUpdateMs) < VEHICLE_DT_MS) return;

    uint16_t dt_ms = (uint16_t)(now - _lastUpdateMs);
    _lastUpdateMs = now;

    float vx_chassis = 0.0f;
    float vy_chassis = 0.0f;
    float wz_chassis = 0.0f;

    _chassis.update(
        _vx,
        _vy,
        _wz,
        vx_chassis,
        vy_chassis,
        wz_chassis
    );

    float vx_korr = 0.0f;
    float vy_korr = 0.0f;
    float wz_korr = 0.0f;

    if (_turnOnly)
    {
        vx_korr = 0.0f;
        vy_korr = 0.0f;
        wz_korr = _regler.updateWz(wz_chassis, _wz_ist, dt_ms);
    }
    else
    {
        MecanumKinematics::limitTranslation(vx_chassis, vy_chassis);

        vx_korr = _regler.updateVx(vx_chassis, _vx_ist, dt_ms);
        vy_korr = _regler.updateVy(vy_chassis, _vy_ist, dt_ms);
        wz_korr = _regler.updateWz(wz_chassis, _wz_ist, dt_ms);

        MecanumKinematics::limitTranslation(vx_korr, vy_korr);
    }

    applyMixer(vx_korr, vy_korr, wz_korr);
}

void VehicleController::applyMixer(float vx_cms, float vy_cms, float wz_rad_s)
{
    MecanumKinematics::inverse(
        vx_cms,
        vy_cms,
        wz_rad_s,
        _wheelSoll
    );
}

void VehicleController::stop()
{
    _vx = 0.0f;
    _vy = 0.0f;
    _wz = 0.0f;

    _turnOnly = false;

    _regler.reset();
    _chassis.reset();

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        _wheelSoll[i] = 0.0f;
    }
}

float VehicleController::getWheelSoll(WheelVehicle w) const
{
    return _wheelSoll[w];
}