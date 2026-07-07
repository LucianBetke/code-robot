// ============================================================
// File: VehicleController.h
// Zweck:
//  - Direkte CMDP -> Rad-Sollwert-Mischung fuer Mecanum
//  - KEINE Vehicle-PI-Regelung
//  - KEINE Phi-/Chassis-Korrektur
//  - Odometrie bleibt separat in MecanumOdometer erhalten
//
// Einheit:
//  - vx, vy, Radgeschwindigkeiten: cm/s
//  - wz: rad/s
// ============================================================

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <Arduino.h>

#include "src/RobotConfig.h"
#include "src/WheelValues.h"

class VehicleController
{
public:
    void begin();

    // Befehl vom CommandRunner:
    // Direkte Umrechnung in Rad-Sollgeschwindigkeiten.
    // Keine geschlossene Fahrzeugregelung.
    void cmd(float vx_cms, float vy_cms, float wz_rad_s);

    // Rueckrechnung der Ist-Radgeschwindigkeiten fuer Diagnose/CHASSIS-Ausgabe.
    // Diese Werte greifen nicht mehr in die Rad-Sollwerte ein.
    void updateIst(const WheelSpeedCms& wheelIst);

    // Keine Vehicle-Regelung mehr.
    // Bleibt nur als leerer Hook fuer den bestehenden FrontApp-Ablauf.
    void update(uint32_t now);

    void stop();

    float getWheelSoll(WheelVehicle w) const;

    float vxIst() const { return _vx_ist; }
    float vyIst() const { return _vy_ist; }
    float wzIst() const { return _wz_ist; }

    float vxSoll() const { return _vx; }
    float vySoll() const { return _vy; }
    float wzSoll() const { return _wz; }

private:
    void applyDriveMode(float& vx_cms, float& vy_cms, float wz_rad_s);
    void applyMixer(float vx_cms, float vy_cms, float wz_rad_s);

    float _vx = 0.0f;
    float _vy = 0.0f;
    float _wz = 0.0f;

    float _vx_ist = 0.0f;
    float _vy_ist = 0.0f;
    float _wz_ist = 0.0f;

    float _wheelSoll[WHEEL_VEHICLE_COUNT] = { 0.0f };

    bool _turnOnly = false;
};

#endif // VEHICLE_CONTROLLER_H