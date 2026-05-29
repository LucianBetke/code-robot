// ============================================================
// File: VehicleControlConfig.h
// Zweck:
//  - Konfiguration fuer die Vehicle-Control-Ebene
//  - PI-Parameter fuer vx, vy und wz
//
// Einheiten:
//  - vx/vy: Fehler in cm/s
//  - wz: Fehler in rad/s
// ============================================================

#ifndef VEHICLE_CONTROL_CONFIG_H
#define VEHICLE_CONTROL_CONFIG_H

#include "src/ControlTypes.h"

struct VehicleControlConfig
{
    PIParam vx;
    PIParam vy;
    PIParam wz;
};

namespace ConfigVehicleFront
{
    constexpr VehicleControlConfig CONFIG =
    {
        { 0.0f, 0.0f },   // vx
        { 0.0f, 0.0f },   // vy
        { 0.0f, 0.0f }    // wz
    };
}

#endif // VEHICLE_CONTROL_CONFIG_H
