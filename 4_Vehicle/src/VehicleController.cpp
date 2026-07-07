// ============================================================
// File: VehicleController.cpp
// Zweck:
//  - Direkte Mecanum-Mischung ohne Vehicle-Regelung
//  - CMDP-Sollwerte werden direkt in Rad-Sollwerte umgesetzt
//  - Odometrie bleibt separat in MecanumOdometer
//
// Koordinatenkonvention:
//  +x  = vorwaerts
//  +y  = links
//  +z  = oben
//  +wz = Drehung gegen den Uhrzeigersinn, von oben betrachtet
//
// Einheiten:
//  vx, vy, Radgeschwindigkeiten: cm/s
//  wz: rad/s
// ============================================================

#include "VehicleController.h"
#include "MecanumKinematics.h"

#include <math.h>

namespace
{
    const float VEHICLE_WZ_EPS = 0.0001f;
}

void VehicleController::begin()
{
    stop();
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

    applyMixer(_vx, _vy, _wz);
}

void VehicleController::applyDriveMode(float& vx_cms, float& vy_cms, float wz_rad_s)
{
    _turnOnly = (fabsf(wz_rad_s) > VEHICLE_WZ_EPS);

    // Wie bisher: Drehbefehle sind reine Drehbefehle.
    // Gemischte Translation + Drehung ist im CommandRunner ohnehin gesperrt.
    if (_turnOnly)
    {
        vx_cms = 0.0f;
        vy_cms = 0.0f;
    }
}

void VehicleController::updateIst(const WheelSpeedCms& wheelIst)
{
    // Nur Diagnose/Rueckrechnung. Keine Sollwert-Korrektur.
    MecanumKinematics::forward(
        wheelIst,
        _vx_ist,
        _vy_ist,
        _wz_ist
    );
}

void VehicleController::update(uint32_t now)
{
    // Keine Vehicle-Regelung mehr.
    // Rad-Sollwerte werden ausschliesslich in cmd() gesetzt.
    (void)now;
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

    _vx_ist = 0.0f;
    _vy_ist = 0.0f;
    _wz_ist = 0.0f;

    _turnOnly = false;

    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        _wheelSoll[i] = 0.0f;
    }
}

float VehicleController::getWheelSoll(WheelVehicle w) const
{
    return _wheelSoll[w];
}