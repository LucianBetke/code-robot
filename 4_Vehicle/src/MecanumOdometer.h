// ============================================================
// File: MecanumOdometer.h
// Zweck:
//  - Mecanum-Odometrie aus vier Rad-Encoderstaenden
//  - Berechnet x/y/phi aus Radwegen
//  - Grundlage fuer CMDP(vx, vy, wz) p
//  - Radreihenfolge kommt aus RobotConfig.h / WheelValues.h:
//      VoRe, VoLi, HiLi, HiRe
// ============================================================

#ifndef MECANUM_ODOMETER_H
#define MECANUM_ODOMETER_H

#include <stdint.h>

#include "src/RobotConfig.h"
#include "src/WheelValues.h"

class MecanumOdometer
{
public:
    MecanumOdometer();

    void reset(const WheelCounts& counts);

    bool update(const WheelCounts& counts);

    bool isPrimed() const { return _primed; }

    float xMm() const { return _x_mm; }
    float yMm() const { return _y_mm; }
    float absMm() const { return _abs_mm; }

    float xCm() const { return _x_mm * 0.1f; }
    float yCm() const { return _y_mm * 0.1f; }
    float absCm() const { return _abs_mm * 0.1f; }

    float phiRad() const { return _phi_rad; }
    float phiDeg() const;

private:
    float countsToMm(int32_t dCounts) const;

    bool _primed;

    int32_t _lastCounts[WHEEL_VEHICLE_COUNT];

    float _x_mm;
    float _y_mm;
    float _abs_mm;
    float _phi_rad;
};

#endif // MECANUM_ODOMETER_H