// ============================================================
// File: MecanumOdometer.h
// Zweck:
//  - Mecanum-Odometrie aus vier Rad-Encoderständen
//  - Berechnet x/y/phi aus Radwegen
//  - Grundlage fuer CMDP(vx, vy, wz) p
// ============================================================

#ifndef MECANUM_ODOMETER_H
#define MECANUM_ODOMETER_H

#include <stdint.h>

class MecanumOdometer
{
public:
    MecanumOdometer();

    void reset(
        int32_t voReCnt,
        int32_t voLiCnt,
        int32_t hiLiCnt,
        int32_t hiReCnt);

    bool update(
        int32_t voReCnt,
        int32_t voLiCnt,
        int32_t hiLiCnt,
        int32_t hiReCnt);

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

    int32_t _lastVoReCnt;
    int32_t _lastVoLiCnt;
    int32_t _lastHiLiCnt;
    int32_t _lastHiReCnt;

    float _x_mm;
    float _y_mm;
    float _abs_mm;
    float _phi_rad;
};

#endif // MECANUM_ODOMETER_H