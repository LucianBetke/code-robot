// ============================================================
// VehicleRegler.h
// ============================================================
#ifndef VEHICLE_REGLER_H
#define VEHICLE_REGLER_H

#include <Arduino.h>

class VehicleRegler
{
public:
    VehicleRegler();
    void setParams(float Kp_vx, float Ki_vx,
        float Kp_vy, float Ki_vy,
        float Kp_wz, float Ki_wz);
    void reset();

    float updateVx(float soll, float ist, uint16_t dt_ms);
    float updateVy(float soll, float ist, uint16_t dt_ms);
    float updateWz(float soll, float ist, uint16_t dt_ms);

private:
    float _Kp_vx, _Ki_vx;
    float _Kp_vy, _Ki_vy;
    float _Kp_wz, _Ki_wz;

    float _integral_vx;
    float _integral_vy;
    float _integral_wz;
};

#endif // VEHICLE_REGLER_H