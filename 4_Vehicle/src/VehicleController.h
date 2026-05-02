// ============================================================
// VehicleController.h
// Fahrzeug-Ebene:
//  - cmd(vx, vy, wz)
//  - Mecanum Mixer
//  - wheelSoll[4]
// ============================================================
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "src/globals.h"

class VehicleController
{
public:
    // Fahrbefehl setzen
    void cmd(float vx, float vy, float wz);
    // Sollwert eines Rades holen
    float getWheelSoll(WheelVehicle w) const;
    // Sollwerte auf vordere Räder schreiben
    void applyFrontWheels();
    // Alle Räder stoppen
    void stop();

private:
    // Mecanum Mixer berechnen
    void update();
    // Fahrzeuggeschwindigkeit
    float _vx = 0.0f;
    float _vy = 0.0f;
    float _wz = 0.0f;
    // Sollgeschwindigkeit pro Rad
    float _wheelSoll[WHEEL_VEHICLE_COUNT] = { 0 };
};

#endif